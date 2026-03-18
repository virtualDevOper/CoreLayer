#include "PCH.h"
#include "NetProjectile3DOF.h"
#include "CONSTANTS.h"

template<typename metricType>
NetProjectile3DOF<metricType>::NetProjectile3DOF(
    std::shared_ptr<DynamicParametersProvider<metricType>> params_provider,
    std::shared_ptr<AbstractWorldModel<metricType>>        world,
    std::shared_ptr<NetAeroModel<metricType>>              aero_model
) : params_provider_(std::move(params_provider)),
    world_(std::move(world)),
    aero_model_(std::move(aero_model)),
    last_computed_force_enu_(Eigen::Vector3<metricType>::Zero()),
    last_cx_(metricType(0)),
    last_mach_(metricType(0)),
    last_rho_(metricType(0)),
    last_s_ref_(metricType(0)),
    last_dynamic_pressure_(metricType(0))
{
    if (!params_provider_ || !aero_model_) {
        throw std::invalid_argument("NetProjectile3DOF: parameters provider and aero model are required");
    }
}

template<typename metricType>
std::string NetProjectile3DOF<metricType>::get_description() const {
    return "Net Projectile 3DOF (translational only, net faces flow)";
}

template<typename metricType>
metricType NetProjectile3DOF<metricType>::computeAirDensity(
    const Eigen::Vector3<metricType>& position) const
{
    auto world = world_.lock();
    if (!world || !world->getAtmosphericModel()) {
        return PhysicsConstants::rho0;
    }
    metricType rho = world->getAtmosphericModel()->getDensity(position);
    return (rho > metricType(1e-8)) ? rho : PhysicsConstants::rho0;
}

template<typename metricType>
metricType NetProjectile3DOF<metricType>::computeMachNumber(
    metricType V_magnitude,
    const Eigen::Vector3<metricType>& position) const
{
    auto world = world_.lock();
    if (!world || !world->getAtmosphericModel()) {
        return V_magnitude / world->getAtmosphericModel()->getSpeedOfSound(position);
    }
    metricType a = world->getAtmosphericModel()->getSpeedOfSound(position);
    return (a > metricType(1e-6)) ? (V_magnitude / a) : metricType(0);
}

template<typename metricType>
std::unique_ptr<KinematicStateDerivative<metricType>>
NetProjectile3DOF<metricType>::get_rhs_derivatives(
    const KinematicState<metricType>& state,
    metricType t)
{
    // === 1. Критичные параметры (Strict) ===
    metricType mass = params_provider_->getMassStrict(t);

    const auto& V_enu = state.getVelocity();
    const auto& position = state.getPosition();

    // === 2. Атмосфера и кинематика ===
    metricType rho = computeAirDensity(position);
    metricType V_mag = V_enu.norm();
    metricType mach = computeMachNumber(V_mag, position);
    metricType q_dyn = metricType(0.5) * rho * V_mag * V_mag;

    // === 3. Аэродинамика ===
    metricType alpha_deg = metricType(0);  // для 3DOF с "парашютным" эффектом
    metricType beta_deg = metricType(0);

    // Получаем Cx и S_ref из модели
    metricType cx = aero_model_->getCx(alpha_deg, t);
    metricType s_ref = aero_model_->getSRef(t);

    // Сила сопротивления: всегда против вектора скорости
    Eigen::Vector3<metricType> F_aero_enu = Eigen::Vector3<metricType>::Zero();
    if (V_mag > metricType(1e-8) && s_ref > metricType(1e-8)) {
        metricType drag_magnitude = cx * q_dyn * s_ref;
        F_aero_enu = -drag_magnitude * (V_enu / V_mag);
    }

    // === 4. Гравитация ===
    Eigen::Vector3<metricType> F_gravity(metricType(0), metricType(0), -mass * PhysicsConstants::g);

    // === 5. Итоговые силы и ускорение ===
    Eigen::Vector3<metricType> F_total = F_aero_enu + F_gravity;
    Eigen::Vector3<metricType> dV_dt = F_total / mass;

    // === 6. Кэширование для snapshot ===
    last_computed_force_enu_ = F_total;
    last_cx_ = cx;
    last_mach_ = mach;
    last_rho_ = rho;
    last_s_ref_ = s_ref;
    last_dynamic_pressure_ = q_dyn;


    // === 7. Возврат производных (3DOF: только поступательное движение) ===
    return std::make_unique<KinematicStateDerivative<metricType>>(
        V_enu,                                  // dPosition/dt
        dV_dt,                                  // dVelocity/dt
        Eigen::Vector3<metricType>::Zero(),    // dEuler/dt = 0
        Eigen::Vector3<metricType>::Zero()     // dAngularVelocity/dt = 0
    );
}

template<typename metricType>
std::unique_ptr<ObjSnapshot<metricType>>
NetProjectile3DOF<metricType>::augmentSnapshot(
    const KinematicState<metricType>& kinematics,
    metricType t) const
{
    // Безопасное получение массы (fallback на 0 для snapshot)
    metricType mass = params_provider_->tryGetMass(t).value_or(metricType(0));



    return ObjSnapshot<metricType>::createBuilder(kinematics)
        .setTime(t)
        .setMass(mass)
        .setTotalForce(last_computed_force_enu_)
        .setAeroCx(last_cx_)
        .setAeroMach(last_mach_)
        .buildUnique();
}

// Явная инстанциация
template class NetProjectile3DOF<GLOBAL_CONFIG::PROJECT_TYPE>;