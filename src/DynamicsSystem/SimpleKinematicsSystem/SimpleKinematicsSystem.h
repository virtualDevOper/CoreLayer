#pragma once
#include "PCH.h"
#include "../IDynamicsSystem.h"

template <typename metricType>
class SimpleKinematicsSystem final : public IDynamicsSystem<metricType> {
public:
    std::string get_description() const override {
        return "Простая кинематическая система: dx/dt = v, dv/dt = 0";
    }

    std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType /*t*/
    ) override {
        // Только кинематика участвует в РК4
        return std::make_unique<KinematicState<metricType>>(
            KinematicState<metricType>::createBuilder()
                .setPosition(state.getVelocity())      // dx/dt = v
                .setVelocity(Eigen::Vector3<metricType>::Zero()) // dv/dt = 0
                .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                .build()
        );
    }

    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType /*t*/
    ) const override {
        // Параметры остаются постоянными (упрощённая модель)
        static const metricType default_mass = 1.0f;
        static const Eigen::Vector3<metricType> default_inertia = Eigen::Vector3<metricType>(1.0f, 1.0f, 1.0f);

        return ObjSnapshot<metricType>::createBuilder(kinematics)
            .setMass(default_mass)
            .setInertia(default_inertia)
            .setTotalForce(Eigen::Vector3<metricType>::Zero())
            .setTotalMoment(Eigen::Vector3<metricType>::Zero())
            .buildUnique();
    }
};