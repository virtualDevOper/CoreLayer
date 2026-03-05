#include "PCH.h"
#include "FullRocketODE.h"

template <typename metricType>
FullRocketODE<metricType>::FullRocketODE(
    std::shared_ptr<DynamicParametersProvider<metricType>> params_provider,
    std::shared_ptr<AbstractWorldModel<metricType>> world,
    std::shared_ptr<aero::AerodynamicsModel> aero_model
)
    : params_provider_(params_provider),
      world_(world),
      aero_model_(std::move(aero_model)),
      last_computed_forces_(Eigen::Vector3<metricType>::Zero()),
      last_computed_moments_(Eigen::Vector3<metricType>::Zero()),
      last_alpha_{0},
      last_beta_{0},
      last_mach_{0}
{
    if (!params_provider) {
        throw std::invalid_argument("Parameters provider cannot be null");
    }
    if (!aero_model_) {
        throw std::invalid_argument("Aerodynamics model cannot be null");
    }
}

template <typename metricType>
std::string FullRocketODE<metricType>::get_description() const {
    return "Full rocket ODE system (6DOF rigid body dynamics)";
}

template <typename metricType>
std::unique_ptr<KinematicStateDerivative<metricType>> FullRocketODE<metricType>::get_rhs_derivatives(
    const KinematicState<metricType>& state,
    metricType t
) {
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("DynamicParametersProvider уже был уничтожен");
    }

    // === Получение параметров ===
    // Масса и инерция критичны — используем Strict
    metricType mass = params_provider->getMassStrict(t);
    auto inertia = params_provider->getInertiaStrict(t);

    // Тяга и COM могут отсутствовать — используем fallback на ноль
    auto thrust_body = params_provider->getThrust(t, Eigen::Vector3<metricType>::Zero());
    auto COM = params_provider->getCOM(t, Eigen::Vector3<metricType>::Zero());

    const auto& V_ENU = state.getVelocity();
    const auto& euler = state.getEulerAngles();

    auto euler_angles = core::math::EulerAngles::ZYX(euler(0), euler(1), euler(2));
    const auto dcm = core::math::eulerToDCM(euler_angles, core::math::CoordinateFrame::ENU);
    auto V_body = core::math::rotateVector(V_ENU, dcm, core::math::CoordinateFrame::ENU, false);

    const auto& angularVelocity = state.getAngularVelocity();

    metricType V_magnitude = V_body.norm();
    metricType alpha_p = computeSpaceAngleOfAttack(V_body);
    [[maybe_unused]] metricType phi_p = computeAerodynamicRollAngle(V_body);
    metricType mach = computeMachNumber(V_magnitude, state.getPosition());
    metricType rho = computeAirDensity(state.getPosition());
    metricType beta_p = computeSideslipAngle(V_body);

    metricType alpha_deg = alpha_p * 180.0 / MathConstants::PI;
    metricType beta_deg = beta_p * 180.0 / MathConstants::PI;

    last_alpha_ = alpha_deg;
    last_beta_ = beta_deg;
    last_mach_ = mach;

    aero::AeroState aero_state;
    aero_state.V = static_cast<double>(V_magnitude);
    aero_state.alpha = static_cast<double>(alpha_deg);
    aero_state.beta = static_cast<double>(beta_deg);
    aero_state.p = static_cast<double>(angularVelocity(0));
    aero_state.q = static_cast<double>(angularVelocity(1));
    aero_state.r = static_cast<double>(angularVelocity(2));
    aero_state.rho = static_cast<double>(rho);
    aero_state.M = static_cast<double>(mach);
    aero_state.dt = static_cast<double>(0.1);
    aero_state.x_com = static_cast<double>(COM[0]);
    aero_state.y_com = static_cast<double>(COM[1]);
    aero_state.z_com = static_cast<double>(COM[2]);

    aero::AeroOutput aero_output;
    try {
        aero_output = aero_model_->calculate(aero_state);
        last_aero_output_ = aero_output;
    } catch (const aero::AeroException& e) {
        std::cerr << "Aerodynamic calculation error: " << e.what() << std::endl;
        aero_output = aero::AeroOutput{};
        last_aero_output_ = aero_output;
    }

    const auto& config = aero_model_->getConfig();
    auto [aero_forces_body_vec, aero_moments_body_vec] = aero_output.getForcesAndMoments(
        config.global.S_ref,
        config.global.c_ref,
        config.global.b_ref,
        static_cast<double>(V_magnitude),
        static_cast<double>(rho)
    );

    Eigen::Vector3<metricType> aero_forces_body(
        static_cast<metricType>(aero_forces_body_vec(0)),
        static_cast<metricType>(aero_forces_body_vec(1)),
        static_cast<metricType>(aero_forces_body_vec(2))
    );

    Eigen::Vector3<metricType> aero_moments_body(
        static_cast<metricType>(aero_moments_body_vec(0)),
        static_cast<metricType>(aero_moments_body_vec(1)),
        static_cast<metricType>(aero_moments_body_vec(2))
    );

    auto F_sum_ENU = computeTotalForcesENU(aero_forces_body, thrust_body, mass, dcm);
    last_computed_forces_ = F_sum_ENU;
    last_computed_moments_ = aero_moments_body;

    auto dV_ENU = computeLinearAccelerationENU(F_sum_ENU, mass);
    auto dEuler_dt = computeEulerAnglesDerivatives(euler, angularVelocity);

    const metricType q_dyn = static_cast<metricType>(0.5 * rho * V_magnitude * V_magnitude);

    // Pitch damping
    aero_moments_body(1) += static_cast<metricType>(aero_output.dmy_dq * angularVelocity(1))
                            * q_dyn * config.global.S_ref * config.global.c_ref;
    // Yaw damping
    aero_moments_body(2) += static_cast<metricType>(aero_output.dmz_dr * angularVelocity(2))
                            * q_dyn * config.global.S_ref * config.global.b_ref;
    // Roll damping
    aero_moments_body(0) += static_cast<metricType>(aero_output.dmx_dp * angularVelocity(0))
                            * q_dyn * config.global.S_ref * config.global.b_ref;

    auto dAngularVelocity_dt = computeAngularAccelerationBody(aero_moments_body, inertia, angularVelocity);

    return std::make_unique<KinematicStateDerivative<metricType>>(
            V_ENU,
            dV_ENU,
            dEuler_dt,
            dAngularVelocity_dt
    );
}

template <typename metricType>
std::unique_ptr<ObjSnapshot<metricType>> FullRocketODE<metricType>::augmentSnapshot(
    const KinematicState<metricType>& state,
    metricType t
) const {
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("DynamicParametersProvider expired in augmentSnapshot");
    }

    // Безопасное получение с fallback на ноль для snapshot
    auto mass = params_provider->tryGetMass(t).value_or(metricType{0});
    auto inertia = params_provider->tryGetInertia(t).value_or(Eigen::Vector3<metricType>::Zero());

    return ObjSnapshot<metricType>::createBuilder(state)
        .setTime(t)
        .setMass(mass)
        .setInertia(inertia)
        .setTotalForce(last_computed_forces_)
        .setTotalMoment(last_computed_moments_)
        .setAeroCx(static_cast<metricType>(last_aero_output_.Cx))
        .setAeroCy(static_cast<metricType>(last_aero_output_.Cy))
        .setAeroCz(static_cast<metricType>(last_aero_output_.Cz))
        .setAeroMx(static_cast<metricType>(last_aero_output_.mx))
        .setAeroMy(static_cast<metricType>(last_aero_output_.my))
        .setAeroMz(static_cast<metricType>(last_aero_output_.mz))
        .setAeroAlpha(last_alpha_)
        .setAeroBeta(last_beta_)
        .setAeroMach(last_mach_)
        .setAeroXcp(static_cast<metricType>(last_aero_output_.x_cp))
        .setAeroStaticMargin(static_cast<metricType>(last_aero_output_.static_margin))
        .buildUnique();
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const {
    metricType v_x = V_body(0);
    metricType v_z = V_body(2);
    if (std::abs(v_x) < 1e-6) {
        return (v_z > 0) ? (MathConstants::PI/2) : -(MathConstants::PI/2);
    }
    return std::atan2(-v_z, v_x);
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeSideslipAngle(const Eigen::Vector3<metricType>& V_body) const {
    metricType v_x = V_body(0);
    metricType v_y = V_body(1);
    if (std::abs(v_x) < 1e-6) {
        return (v_y > 0) ? (MathConstants::PI/2) : -(MathConstants::PI/2);
    }
    return std::atan2(v_y, v_x);
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeAerodynamicRollAngle(const Eigen::Vector3<metricType>& V_body) const {
    metricType v_y = V_body(1);
    metricType v_z = V_body(2);
    metricType denominator = std::sqrt(v_y*v_y + v_z*v_z);
    if (denominator < 1e-6) return 0.0;
    return std::atan2(v_z, -v_y);
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeMachNumber(metricType V_magnitude, const Eigen::Vector3<metricType>& position) const {
    auto world = world_.lock();
    if (!world || !world->getAtmosphericModel()) {
        throw std::runtime_error("World or atmospheric model not available");
    }
    return V_magnitude / world->getAtmosphericModel()->getSpeedOfSound(position);
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeAirDensity(const Eigen::Vector3<metricType>& position) const {
    auto world = world_.lock();
    if (!world || !world->getAtmosphericModel()) {
        throw std::runtime_error("World or atmospheric model not available");
    }
    metricType rho = world->getAtmosphericModel()->getDensity(position);
    if (rho <= 0.0) {
        rho = static_cast<metricType>(PhysicsConstants::rho0);
    }
    return rho;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeTotalForcesENU(
    const Eigen::Vector3<metricType>& F_aero_body,
    const Eigen::Vector3<metricType>& F_thrust_body,
    metricType mass,
    const core::math::Matrix3& dcm) const
{
    Eigen::Vector3<metricType> gravity_ENU(0, 0, -mass * PhysicsConstants::g);
    auto F_aero_ENU = core::math::rotateVector(F_aero_body, dcm, core::math::CoordinateFrame::ENU, true);
    auto F_thrust_ENU = core::math::rotateVector(F_thrust_body, dcm, core::math::CoordinateFrame::ENU, true);
    return F_aero_ENU + F_thrust_ENU + gravity_ENU;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeLinearAccelerationENU(
    const Eigen::Vector3<metricType>& F_sum,
    metricType mass) const
{
    return F_sum / mass;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeEulerAnglesDerivatives(
    const Eigen::Vector3<metricType>& euler,
    const Eigen::Vector3<metricType>& angularVelocity) const
{
    auto euler_angles = core::math::EulerAngles::ZYX(euler(0), euler(1), euler(2));
    return core::math::angularVelocityToEulerRates(
        euler_angles,
        angularVelocity,
        core::math::CoordinateFrame::ENU
    );
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeAngularAccelerationBody(
    const Eigen::Vector3<metricType>& M_sum,
    const Eigen::Vector3<metricType>& inertia,
    const Eigen::Vector3<metricType>& angularVelocity) const
{
    metricType I_x = inertia(0);
    metricType I_y = inertia(1);
    metricType I_z = inertia(2);
    metricType w_x = angularVelocity(0);
    metricType w_y = angularVelocity(1);
    metricType w_z = angularVelocity(2);
    metricType M_x = M_sum(0);
    metricType M_y = M_sum(1);
    metricType M_z = M_sum(2);
    
    Eigen::Vector3<metricType> dOmega_dt;
    dOmega_dt(0) = (M_x - (I_z - I_y) * w_y * w_z) / I_x;
    dOmega_dt(1) = (M_y - (I_x - I_z) * w_x * w_z) / I_y;
    dOmega_dt(2) = (M_z - (I_y - I_x) * w_x * w_y) / I_z;
    return dOmega_dt;
}

// Explicit template instantiation
template class FullRocketODE<GLOBAL_CONFIG::PROJECT_TYPE>;