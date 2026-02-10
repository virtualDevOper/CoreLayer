#include "PCH.h"
#include "FullRocketODE.h"

template <typename metricType>
FullRocketODE<metricType>::FullRocketODE(
    const std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
    const std::shared_ptr<AbstractWorldModel<metricType>> world
)
    : params_provider_(params_provider),
      world_(world),
      last_computed_forces_(Eigen::Vector3<metricType>::Zero()),
      last_computed_moments_(Eigen::Vector3<metricType>::Zero())
{
    if (params_provider_.expired()) {
        throw std::invalid_argument("Parameters provider cannot be null");
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
    auto V_body = core::math::rotateVector(V_ENU, dcm, core::math::CoordinateFrame::ENU, false); // body_to_enu = false, так как матрицу нужно траснспонировать

    const auto& angularVelocity = state.getAngularVelocity();
    // тут стопрацентная ошибка у нас, так как поменяли системы координат
    metricType alpha_p = computeSpaceAngleOfAttack(V_body);
    metricType phi_p = computeAerodynamicRollAngle(V_body);
    metricType mach = computeMachNumber(V_body.norm(), Eigen::Vector3<metricType>::Zero()); // TODO: pass real position
    metricType rho = computeAirDensity(state.getPosition());

    auto aero_model = params_provider->getAeroModel();
    /*
    auto aero_forces_body = aero_model->computeAerodynamicForces(V_body, rho, mach);
    auto aero_moments_body = aero_model->computeAerodynamicMoments(V_body, omega, rho, mach, euler);
    */

    auto aero_forces_body = Eigen::Vector3<metricType>::Zero();
    auto aero_moments_body = Eigen::Vector3<metricType>::Zero();




    auto thrust_body = params_provider->getThrust(t);
    //auto thrust_body = Eigen::Vector3<metricType>::Zero();
    // Thrust is correct at the beginning, just check forces for coordinate system
    auto F_sum_ENU = computeTotalForcesENU(aero_forces_body, thrust_body, mass, dcm);
    last_computed_forces_ = F_sum_ENU;

    auto dV_ENU = computeLinearAccelerationENU(F_sum_ENU, mass);
    auto dEuler_dt = computeEulerAnglesDerivatives(euler, angularVelocity);
    auto dAngularVelocity_dt = computeAngularAccelerationBody(aero_moments_body, inertia, angularVelocity);

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
            .buildUnique();
}




template <typename metricType>
metricType FullRocketODE<metricType>::computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const {
    metricType v_x = V_body(0);
    metricType v_y = V_body(1);
    metricType v_z = V_body(2);
    metricType numerator = std::sqrt(v_y*v_y + v_z*v_z);
    metricType denominator = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
    if (denominator < 1e-6) return 0.0;
    return std::asin(numerator / denominator);
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
    return world->getAtmosphericModel()->getDensity(position);
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