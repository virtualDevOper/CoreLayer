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
std::unique_ptr<KinematicState<metricType>> FullRocketODE<metricType>::get_rhs_derivatives(
    const KinematicState<metricType>& kinematics,
    metricType t
) {
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("DynamicParametersProvider has been destroyed");
    }
    
    metricType mass = params_provider->getMass(t);
    auto inertia = params_provider->getInertia(t);

    const auto& V_earth = kinematics.getVelocity();
    const auto& euler = kinematics.getEulerAngles();
    auto V_body = transformVelocityEarthToBody(V_earth, euler);

    const auto& omega = kinematics.getAngularVelocity();

    // Compute aerodynamic forces and moments
    metricType alpha_p = computeSpaceAngleOfAttack(V_body);
    metricType phi_p = computeAerodynamicRollAngle(V_body);
    metricType mach = computeMachNumber(V_body.norm(), Eigen::Vector3<metricType>::Zero()); // TODO: pass real position
    metricType rho = computeAirDensity(Eigen::Vector3<metricType>::Zero()); // TODO: pass real position

    auto aero_model = params_provider->getAeroModel();
    auto aero_forces_body = aero_model->computeAerodynamicForces(V_body, rho, mach);
    auto aero_moments_body = aero_model->computeAerodynamicMoments(V_body, omega, rho, mach, euler);

    last_computed_forces_ = aero_forces_body;
    last_computed_moments_ = aero_moments_body;

    auto thrust_body = params_provider->getThrust(t);

    auto F_sum_body = computeTotalForces(aero_forces_body, thrust_body, mass, euler);
    last_computed_forces_ = F_sum_body;

    // Compute derivatives
    auto dV_body_dt = computeLinearAccelerationBody(V_body, omega, F_sum_body, mass);
    auto dV_earth_dt = transformAccelerationBodyToEarth(dV_body_dt, V_body, omega, euler);
    auto dEuler_dt = computeEulerAnglesDerivatives(euler, omega);
    auto dOmega_dt = computeAngularAccelerationBody(aero_moments_body, inertia, omega);

    return std::make_unique<KinematicState<metricType>>(
        KinematicState<metricType>::createBuilder()
            .setPosition(kinematics.getVelocity())  // dx/dt = v
            .setVelocity(dV_earth_dt)               // dv/dt = computed acceleration
            .setEulerAngles(dEuler_dt)              // dEuler/dt
            .setAngularVelocity(dOmega_dt)          // dOmega/dt
            .build()
    );
}

template <typename metricType>
std::unique_ptr<ObjSnapshot<metricType>> FullRocketODE<metricType>::augmentSnapshot(
    const KinematicState<metricType>& kinematics,
    metricType t
) const {
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("DynamicParametersProvider expired in augmentSnapshot");
    }

    // Transform forces and moments from body to earth frame for logging
    const auto& euler = kinematics.getEulerAngles();
    std::array<metricType, 3> F_body_arr = {
        last_computed_forces_.x(),
        last_computed_forces_.y(),
        last_computed_forces_.z()
    };
    auto F_earth_arr = TransformationFactory<metricType>::createBodyToEarthTransform(
        euler.x(), euler.y(), euler.z(), F_body_arr)->result_getter();
    Eigen::Vector3<metricType> F_earth(F_earth_arr[0], F_earth_arr[1], F_earth_arr[2]);

    std::array<metricType, 3> M_body_arr = {
        last_computed_moments_.x(),
        last_computed_moments_.y(),
        last_computed_moments_.z()
    };
    auto M_earth_arr = TransformationFactory<metricType>::createBodyToEarthTransform(
        euler.x(), euler.y(), euler.z(), M_body_arr)->result_getter();
    Eigen::Vector3<metricType> M_earth(M_earth_arr[0], M_earth_arr[1], M_earth_arr[2]);

    return ObjSnapshot<metricType>::createBuilder(kinematics)
        .setTime(t)
        .setMass(params_provider->getMass(t))
        .setInertia(params_provider->getInertia(t))
        .setTotalForce(last_computed_forces_)
        .setTotalMoment(last_computed_moments_)
        .setTotalForceEarth(F_earth)
        .setTotalMomentEarth(M_earth)
        .buildUnique();
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::transformVelocityEarthToBody(
    const Eigen::Vector3<metricType>& V_earth,
    const Eigen::Vector3<metricType>& euler) const
{
    std::array<metricType, 3> V_earth_array = {V_earth(0), V_earth(1), V_earth(2)};
    auto transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
        euler(0), euler(1), euler(2), V_earth_array);
    auto result = transform->result_getter();
    return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::transformAccelerationBodyToEarth(
    const Eigen::Vector3<metricType>& a_body,
    const Eigen::Vector3<metricType>& V_body,
    const Eigen::Vector3<metricType>& omega,
    const Eigen::Vector3<metricType>& euler) const
{
    std::array<metricType, 3> a_body_array = {a_body(0), a_body(1), a_body(2)};
    auto transform = std::make_unique<Svyaz_to_zemn_Direction<metricType>>(
        euler(0), euler(1), euler(2), a_body_array);
    auto result = transform->result_getter();
    return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
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
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeTotalForces(
    const Eigen::Vector3<metricType>& F_aero,
    const Eigen::Vector3<metricType>& F_thrust,
    metricType mass,
    const Eigen::Vector3<metricType>& euler) const
{
    std::array<metricType, 3> gravity_array = {0, -mass * PhysicsConstants::g, 0};
    auto gravity_transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
        euler(0), euler(1), euler(2), gravity_array);
    auto gravity_body_array = gravity_transform->result_getter();
    Eigen::Vector3<metricType> gravity_body(
        gravity_body_array[0], gravity_body_array[1], gravity_body_array[2]);
    return F_aero + F_thrust + gravity_body;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeLinearAccelerationBody(
    const Eigen::Vector3<metricType>& V_body,
    const Eigen::Vector3<metricType>& omega,
    const Eigen::Vector3<metricType>& F_sum,
    metricType mass) const
{
    metricType v_x = V_body(0), v_y = V_body(1), v_z = V_body(2);
    metricType w_x = omega(0), w_y = omega(1), w_z = omega(2);
    Eigen::Vector3<metricType> dV_dt;
    dV_dt(0) = -w_y * v_z + w_z * v_y + F_sum(0) / mass;
    dV_dt(1) = -w_z * v_x + w_x * v_z + F_sum(1) / mass;
    dV_dt(2) = -w_x * v_y + w_y * v_x + F_sum(2) / mass;
    return dV_dt;
}

template <typename metricType>
Eigen::Vector3<metricType> FullRocketODE<metricType>::computeEulerAnglesDerivatives(
    const Eigen::Vector3<metricType>& euler,
    const Eigen::Vector3<metricType>& omega) const
{
    metricType theta = euler(0);  // pitch
    metricType psi = euler(1);    // yaw
    metricType gamma = euler(2);  // roll
    metricType w_x = omega(0);
    metricType w_y = omega(1);
    metricType w_z = omega(2);
    metricType cos_theta = std::cos(theta);
    metricType sin_theta = std::sin(theta);
    metricType cos_gamma = std::cos(gamma);
    metricType sin_gamma = std::sin(gamma);
    metricType tan_theta = std::tan(theta);
    
    Eigen::Vector3<metricType> dEuler_dt;
    if (std::abs(cos_theta) > 1e-6) {
        dEuler_dt(1) = (w_y * cos_gamma - w_z * sin_gamma) / cos_theta;  // dPsi/dt
    } else {
        dEuler_dt(1) = 0.0;
    }
    dEuler_dt(2) = w_x - tan_theta * (w_y * cos_gamma - w_z * sin_gamma);  // dGamma/dt
    dEuler_dt(0) = w_y * sin_gamma + w_z * cos_gamma;  // dTheta/dt
    
    if (!std::isfinite(dEuler_dt(0)) || !std::isfinite(dEuler_dt(1)) || !std::isfinite(dEuler_dt(2))) {
        throw std::runtime_error("Non-finite values in Euler angles derivatives");
    }
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