//
// Created by 4NR_Operator_3 on 16.12.2025.
//

#pragma once
#include "../IDynamicsSystem.h"

template <typename metricType>
class SimpleKinematicsSystem final : public IDynamicsSystem<metricType> {
public:
    std::string get_description() override {
        return "Простая кинематическая система: dx/dt = v, dv/dt = 0";
    }

    std::unique_ptr<ObjSnapshot<metricType>> get_rhs_derivatives(
        const ObjSnapshot<metricType>& previous_state,
        metricType t) override {

        // Для простого объекта считаем только производные позиции = скорость
        // Все остальные производные = 0

        return ObjSnapshot<metricType>::createBuilder()
            .setPosition(previous_state.getVelocity())      // dx/dt = v
            .setVelocity(Eigen::Vector3<metricType>::Zero()) // dv/dt = 0 (без сил)
            .setEulerAngles(Eigen::Vector3<metricType>::Zero())
            .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
            .setTotalForce(Eigen::Vector3<metricType>::Zero())
            .setTotalMoment(Eigen::Vector3<metricType>::Zero())
            .setInertia(previous_state.getInertia())  // не меняется
            .setMass(previous_state.getMass())        // не меняется
            .buildUnique();
    }
};