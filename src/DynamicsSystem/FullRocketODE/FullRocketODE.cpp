#include "PCH.h"
#include "FullRocketODE.h"

template <typename metricType>
FullRocketODE<metricType>::FullRocketODE(
    const std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
    const std::shared_ptr<AbstractWorldModel<metricType>> world,
    std::shared_ptr<aero::AerodynamicsModel> aero_model
)
    : params_provider_(params_provider),
      world_(world),
      aero_model_(std::move(aero_model)),
      last_computed_forces_(Eigen::Vector3<metricType>::Zero()),
      last_computed_moments_(Eigen::Vector3<metricType>::Zero())
{
    if (params_provider_.expired()) {
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

    metricType mass = params_provider->getMass(t);
    auto inertia = params_provider->getInertia(t);

    const auto& V_ENU = state.getVelocity();
    const auto& euler = state.getEulerAngles();
    
    // euler(0) = psi (yaw), euler(1) = theta (pitch) , euler(2) = gamma (roll)
    auto euler_angles = core::math::EulerAngles::ZYX(euler(0), euler(1), euler(2));
    const auto dcm = core::math::eulerToDCM(euler_angles, core::math::CoordinateFrame::ENU);
    auto V_body = core::math::rotateVector(V_ENU, dcm, core::math::CoordinateFrame::ENU, false);




    const auto& angularVelocity = state.getAngularVelocity();
    
    // Prepare aerodynamic state
    metricType V_magnitude = V_body.norm();
    metricType alpha_p = computeSpaceAngleOfAttack(V_body);
    metricType phi_p = computeAerodynamicRollAngle(V_body);
    metricType mach = computeMachNumber(V_magnitude, state.getPosition());
    metricType rho = computeAirDensity(state.getPosition());
    metricType beta_p = computeSideslipAngle(V_body);


    // Convert to degrees for aero library
    metricType alpha_deg = alpha_p * 180.0 / M_PI;
    metricType beta_deg = beta_p * 180.0 / M_PI;

    // Сохраняем для snapshot
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
    auto COM = params_provider->getCOM(t);
    aero_state.x_com = static_cast<double>(COM[0]);
    aero_state.y_com = static_cast<double>(COM[1]);
    aero_state.z_com = static_cast<double>(COM[2]);






    aero::AeroOutput aero_output;
    try {
        aero_output = aero_model_->calculate(aero_state);
        last_aero_output_ = aero_output;  // Сохраняем для snapshot
    } catch (const aero::AeroException& e) {
        // Handle aerodynamic calculation errors gracefully
        std::cerr << "Aerodynamic calculation error: " << e.what() << std::endl;
        aero_output = aero::AeroOutput{}; // Use zero coefficients
        last_aero_output_ = aero_output;
    }







    // Convert aerodynamic coefficients to forces and moments in body frame
    const auto& config = aero_model_->getConfig();
    auto [aero_forces_body_vec, aero_moments_body_vec] = aero_output.getForcesAndMoments(
        config.global.S_ref,
        config.global.c_ref,
        config.global.b_ref,
        static_cast<double>(V_magnitude),
        static_cast<double>(rho)
    );

    // Convert from Eigen::Vector3d to Eigen::Vector3<metricType>
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




    auto thrust_body = params_provider->getThrust(t);
    
    // Compute total forces and moments
    auto F_sum_ENU = computeTotalForcesENU(aero_forces_body, thrust_body, mass, dcm);
    last_computed_forces_ = F_sum_ENU;
    last_computed_moments_ = aero_moments_body;

    auto dV_ENU = computeLinearAccelerationENU(F_sum_ENU, mass);
    auto dEuler_dt = computeEulerAnglesDerivatives(euler, angularVelocity);

    const metricType q_dyn = static_cast<metricType>(0.5 * rho * V_magnitude * V_magnitude);


                           /*
                           // === ВСТАВИТЬ ПОСЛЕ aero_model_->calculate(aero_state) ===

                        std::cerr << "=== AERO DEBUG t=" << t << " ===" << std::endl;
                        std::cerr << "V_body: [" << V_body.transpose() << "]" << std::endl;
                        std::cerr << "alpha_deg=" << alpha_deg << ", beta_deg=" << beta_deg << std::endl;
                        std::cerr << "Mach=" << mach << ", rho=" << rho << ", q_dyn=" << q_dyn << std::endl;
                        std::cerr << "Cz=" << aero_output.Cz << ", my=" << aero_output.my << std::endl;
                        std::cerr << "x_cp=" << aero_output.x_cp << ", x_com=" << COM[0] << std::endl;
                        std::cerr << "arm_x=" << (aero_output.x_cp - COM[0]) << std::endl;
                        std::cerr << "static_margin=" << aero_output.static_margin << std::endl;

                        // КРИТИЧЕСКИ ВАЖНО: проверка знака dmy_dalpha
                        std::cerr << "\n--- STATIC DERIVATES ---" << std::endl;
                        std::cerr << "dmy_dalpha=" << aero_output.dmy_dalpha
                                  << " (sign vs alpha: " << (aero_output.dmy_dalpha * alpha_deg < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;
                        std::cerr << "dCz_dalpha=" << aero_output.dCz_dalpha << std::endl;

                        // Демпфирование
                        std::cerr << "\n--- DYNAMIC DERIVATES ---" << std::endl;
                        std::cerr << "dmy_dq=" << aero_output.dmy_dq
                                  << " (sign: " << (aero_output.dmy_dq < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;
                        std::cerr << "dmx_dp=" << aero_output.dmx_dp
                                  << " (sign: " << (aero_output.dmx_dp < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;
                        std::cerr << "dmz_dr=" << aero_output.dmz_dr
                                  << " (sign: " << (aero_output.dmz_dr < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;

                        // Проверка вкладов демпфирования в моменты
                        const metricType damp_my = static_cast<metricType>(aero_output.dmy_dq * angularVelocity(1))
                            * q_dyn * config.global.S_ref * config.global.c_ref;
                        const metricType damp_mx = static_cast<metricType>(aero_output.dmx_dp * angularVelocity(0))
                            * q_dyn * config.global.S_ref * config.global.b_ref;
                        const metricType damp_mz = static_cast<metricType>(aero_output.dmz_dr * angularVelocity(2))
                            * q_dyn * config.global.S_ref * config.global.b_ref;

                        std::cerr << "\n--- DEMPH INPUT ---" << std::endl;
                        std::cerr << "damp_my=" << damp_my << " (against q: "
                                  << (damp_my * angularVelocity(1) < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;
                        std::cerr << "damp_mx=" << damp_mx << " (against p: "
                                  << (damp_mx * angularVelocity(0) < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;
                        std::cerr << "damp_mz=" << damp_mz << " (against r: "
                                  << (damp_mz * angularVelocity(2) < 0 ? "OK ✓" : "FAIL ❌") << ")" << std::endl;

                        std::cerr << "\nangularVelocity: [" << angularVelocity.transpose() << "]" << std::endl;
                        std::cerr << "==================" << std::endl;
                        */


    // Pitch damping (тангаж)
    aero_moments_body(1) += static_cast<metricType>(aero_output.dmy_dq * angularVelocity(1))
                            * q_dyn * config.global.S_ref * config.global.c_ref;

    // Yaw damping (рыскание)
    aero_moments_body(2) += static_cast<metricType>(aero_output.dmz_dr * angularVelocity(2))
                            * q_dyn * config.global.S_ref * config.global.b_ref;

    // Roll damping (крен)
    aero_moments_body(0) += static_cast<metricType>(aero_output.dmx_dp * angularVelocity(0))
                            * q_dyn * config.global.S_ref * config.global.b_ref;



    auto dAngularVelocity_dt = computeAngularAccelerationBody(aero_moments_body, inertia, angularVelocity);

    /*
    std::cerr << "DEBUG: alpha=" << alpha_deg
          << ", Cz=" << aero_output.Cz
          << ", my=" << aero_output.my
          << ", x_cp=" << (aero_output.x_cp)
          << ", static_margin=" << aero_output.static_margin << std::endl;
          */

    return std::make_unique<KinematicStateDerivative<metricType>>(
            V_ENU,  // dPosition/dt = Velocity
            dV_ENU,               // dVelocity/dt = Acceleration
            dEuler_dt,              // dEuler/dt
            dAngularVelocity_dt          // dАngularVelocity/dt
    );


}

template <typename metricType>
std::unique_ptr<ObjSnapshot<metricType>> FullRocketODE<metricType>::augmentSnapshot(
    const KinematicState<metricType>& kinematics,
    metricType t
) const {
    auto params_provider = params_provider_.lock();
    if (!params_provider) {throw std::runtime_error("DynamicParametersProvider expired in augmentSnapshot");}

    return ObjSnapshot<metricType>::createBuilder(kinematics)
        .setTime(t)
        .setMass(params_provider->getMass(t))
        .setInertia(params_provider->getInertia(t))
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

    // Защита от деления на ноль
    if (std::abs(v_x) < 1e-6) {
        return (v_z > 0) ? (MathConstants::PI/2) : -(MathConstants::PI/2);
    }
    return std::atan2(-v_z, v_x);
}

template <typename metricType>
metricType FullRocketODE<metricType>::computeSideslipAngle(
    const Eigen::Vector3<metricType>& V_body) const {

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
    
    // Если плотность <= 0 (например, ракета ушла за пределы модели или врезалась в землю),
    // используем плотность на уровне моря для избежания ошибок в аэродинамике
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
    // Gravity force in ENU (Z up -> -mg)
    Eigen::Vector3<metricType> gravity_ENU(0, 0, -mass * PhysicsConstants::g);

    // Transform aerodynamic and thrust forces from body to ENU
    auto F_aero_ENU = core::math::rotateVector(F_aero_body, dcm, core::math::CoordinateFrame::ENU, true);
    auto F_thrust_ENU = core::math::rotateVector(F_thrust_body, dcm, core::math::CoordinateFrame::ENU, true);

    // Sum all forces in ENU
    return F_aero_ENU + F_thrust_ENU + gravity_ENU;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeLinearAccelerationENU(
    const Eigen::Vector3<metricType>& F_sum,
    metricType mass) const
{
    Eigen::Vector3<metricType> dV_dt;
    dV_dt(0) = F_sum(0) / mass;
    dV_dt(1) = F_sum(1) / mass;
    dV_dt(2) = F_sum(2) / mass;
    return dV_dt;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeEulerAnglesDerivatives(
    const Eigen::Vector3<metricType>& euler,
    const Eigen::Vector3<metricType>& angularVelocity) const
{
    // Use FlightMath library function for Euler rate computation in ENU coordinates
    // euler(0) = psi (yaw), euler(1) = theta (pitch), euler(2) = gamma (roll)
    auto euler_angles = core::math::EulerAngles::ZYX(euler(0), euler(1), euler(2));
    
    // Convert angular velocity to Euler rates using FlightMath function
    // This replaces manual matrix math with library function
    auto dEuler_dt = core::math::angularVelocityToEulerRates(
        euler_angles, 
        angularVelocity, 
        core::math::CoordinateFrame::ENU
    );
    
    return dEuler_dt;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeAngularAccelerationBody(
    const Eigen::Vector3<metricType>& M_sum,
    const Eigen::Vector3<metricType>& inertia,
    const Eigen::Vector3<metricType>& omega) const
{
    metricType I_x = inertia(0);
    metricType I_y = inertia(1);
    metricType I_z = inertia(2);
    metricType w_x = omega(0);
    metricType w_y = omega(1);
    metricType w_z = omega(2);
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