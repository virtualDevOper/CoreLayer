#pragma once

#include "DynamicsSystem/IDynamicsSystem.h"
#include "DynamicsSystem/Aerodynamics/NetAeroModel.h"
#include "utils/ParameterProvider/DynamicParametersProvider.h"
#include "WorldModel/AbstractWorldModel.h"

/**
 * @brief 3DOF-динамика сетевого снаряда (только поступательное движение).
 *
 * Уравнения:
 *   dr/dt = V
 *   dV/dt = (F_aero + F_gravity) / m
 *
 * Сетка всегда ориентирована "лицом" к потоку (парашютный эффект).
 * Нет вращения, моментов, углов Эйлера.
 */
template<typename metricType>
class NetProjectile3DOF final : public IDynamicsSystem<metricType> {
private:
    std::shared_ptr<DynamicParametersProvider<metricType>> params_provider_;
    std::weak_ptr<AbstractWorldModel<metricType>>          world_;
    std::shared_ptr<NetAeroModel<metricType>>              aero_model_;

    // Кэш для augmentSnapshot (по аналогии с FullRocketODE)
    mutable Eigen::Vector3<metricType> last_computed_force_enu_;
    mutable metricType last_cx_;
    mutable metricType last_alpha_;
    mutable metricType last_beta_;
    mutable metricType last_mach_;
    mutable metricType last_rho_;
    mutable metricType last_s_ref_;
    mutable metricType last_dynamic_pressure_;

public:
    explicit NetProjectile3DOF(
        std::shared_ptr<DynamicParametersProvider<metricType>> params_provider,
        std::shared_ptr<AbstractWorldModel<metricType>>        world,
        std::shared_ptr<NetAeroModel<metricType>>              aero_model
    );

    [[nodiscard]] std::string get_description() const override;

    std::unique_ptr<KinematicStateDerivative<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) override;

    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& state,
        metricType t
    ) const override;

private:
    metricType computeAirDensity(const Eigen::Vector3<metricType>& position) const;
    metricType computeMachNumber(metricType V_magnitude, const Eigen::Vector3<metricType>& position) const;
};