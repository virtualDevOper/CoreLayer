/*// src/DynamicsSystem/NetProjectile3DOF/NetProjectile3DOF.h
#pragma once
#include "../IDynamicsSystem.h"
#include "../../WorldModel/AbstractWorldModel.h"
#include "../Aerodynamics/SimplifiedAeroModel.h"

/**
 * @brief 3DOF-динамика сетевого снаряда (только поступательное движение).
 *
 * @details Уравнения:
 *   dr/dt = V
 *   dV/dt = (F_aero + F_gravity) / m
 *
 * Сетка всегда ориентирована "лицом" к потоку (парашютный эффект).
 * Нет вращения, моментов, углов Эйлера.
 #1#
template<typename metricType>
class NetProjectile3DOF final : public IDynamicsSystem<metricType> {
private:
    //std::weak_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider_;
    std::weak_ptr<AbstractWorldModel<metricType>> world_;
    std::shared_ptr<SimplifiedAeroModel<metricType>> aero_model_;

public:
    explicit NetProjectile3DOF(
        //std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
        std::shared_ptr<AbstractWorldModel<metricType>> world,
        std::shared_ptr<SimplifiedAeroModel<metricType>> aero_model
    );

    std::string get_description() const override;

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
};*/