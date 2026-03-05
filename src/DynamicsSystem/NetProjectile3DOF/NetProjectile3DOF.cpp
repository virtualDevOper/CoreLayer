/*// src/DynamicsSystem/NetProjectile3DOF/NetProjectile3DOF.cpp
#include "PCH.h"
#include "NetProjectile3DOF.h"

template<typename metricType>
NetProjectile3DOF<metricType>::NetProjectile3DOF(
    std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
    std::shared_ptr<AbstractWorldModel<metricType>> world,
    std::shared_ptr<SimplifiedAeroModel<metricType>> aero_model
) : params_provider_(params_provider),
    world_(world),
    aero_model_(std::move(aero_model))
{
    if (params_provider_.expired() || !aero_model_) {
        throw std::invalid_argument("Parameters provider and aero model are required");
    }
}

template<typename metricType>
std::string NetProjectile3DOF<metricType>::get_description() const {
    return "Net Projectile 3DOF (translational only, net faces flow)";
}

template<typename metricType>
std::unique_ptr<KinematicStateDerivative<metricType>>
NetProjectile3DOF<metricType>::get_rhs_derivatives(
    const KinematicState<metricType>& state,
    metricType t)
{
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("Parameters provider expired");
    }

    const auto& V_enu = state.getVelocity();
    const auto& position = state.getPosition();

    // === ПАРАМЕТРЫ ===
    metricType mass = params_provider->getMass(t);
    metricType rho = computeAirDensity(position);

    // === АЭРОДИНАМИЧЕСКАЯ СИЛА (в ENU) ===
    Eigen::Vector3<metricType> F_aero = aero_model_->computeForceENU(V_enu, rho, t);

    // === ГРАВИТАЦИЯ (ENU: Z up) ===
    Eigen::Vector3<metricType> F_gravity(0, 0, -mass * PhysicsConstants::g);

    // === УСКОРЕНИЕ ===
    Eigen::Vector3<metricType> F_total = F_aero + F_gravity;
    Eigen::Vector3<metricType> dV_dt = F_total / mass;

    // === ПРОИЗВОДНЫЕ (3DOF: только позиция и скорость) ===
    // Углы и угловая скорость не меняются (константы)
    return std::make_unique<KinematicStateDerivative<metricType>>(
        V_enu,                                    // dPosition/dt
        dV_dt,                                    // dVelocity/dt
        Eigen::Vector3<metricType>::Zero(),       // dEuler/dt = 0
        Eigen::Vector3<metricType>::Zero()        // dAngularVelocity/dt = 0
    );
}

template<typename metricType>
std::unique_ptr<ObjSnapshot<metricType>>
NetProjectile3DOF<metricType>::augmentSnapshot(
    const KinematicState<metricType>& kinematics,
    metricType t) const
{
    auto params_provider = params_provider_.lock();
    if (!params_provider) {
        throw std::runtime_error("Parameters provider expired");
    }

    // Для 3DOF: моменты = 0, углы = константы
    return ObjSnapshot<metricType>::createBuilder(kinematics)
        .setTime(t)
        .setMass(params_provider->getMass(t))
        .setInertia(Eigen::Vector3<metricType>::Zero())  // не используется
        .setTotalForce(Eigen::Vector3<metricType>::Zero()) // можно вычислить, если нужно
        .setTotalMoment(Eigen::Vector3<metricType>::Zero())
        .setAeroCx(metricType(0))   // можно добавить вывод Cx, если нужно
        .setAeroCy(metricType(0))
        .setAeroCz(metricType(0))
        .setAeroMx(metricType(0))
        .setAeroMy(metricType(0))
        .setAeroMz(metricType(0))
        .setAeroAlpha(metricType(0))
        .setAeroBeta(metricType(0))
        .setAeroMach(metricType(0))
        .setAeroXcp(metricType(0))
        .setAeroStaticMargin(metricType(0))
        .buildUnique();
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
    return (rho > metricType(0)) ? rho : PhysicsConstants::rho0;
}

template class NetProjectile3DOF<GLOBAL_CONFIG::PROJECT_TYPE>;*/