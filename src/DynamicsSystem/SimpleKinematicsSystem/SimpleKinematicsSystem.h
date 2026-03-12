#pragma once
#include "DynamicsSystem/IDynamicsSystem.h"

template <typename metricType>
class SimpleKinematicsSystem final : public IDynamicsSystem<metricType> {
public:
    std::string get_description() const override {
        return "Простая кинематическая система: dx/dt = v, dv/dt = 0";
    }

    std::unique_ptr<KinematicStateDerivative<metricType>> get_rhs_derivatives(
    const KinematicState<metricType>& state,
    metricType /*t*/
    ) override {
        return std::make_unique<KinematicStateDerivative<metricType>>(
        state.getVelocity(),                              // dPosition/dt
        Eigen::Vector3<metricType>::Zero(),               // dVelocity/dt
        state.getAngularVelocity(),                       // dEulerAngles/dt
        Eigen::Vector3<metricType>::Zero()                // dAngularVelocity/dt
        );
    }

    // === ИСПРАВЛЕНО: используем Builder вместо конструктора ===
    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
    const KinematicState<metricType>& state,
    metricType t
    ) const override {
        return ObjSnapshot<metricType>::createBuilder(state)
        .setTime(t)
        .buildUnique();
    }
};