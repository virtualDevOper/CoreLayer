//
// Created by 4NR_Operator_3 on 16.12.2025.
//

#pragma once
#include "../AbstractObject.h"
#include "../../DynamicsSystem/SimpleKinematicsSystem/SimpleKinematicsSystem.h"

template<typename metricType>
class SimpleObject final : public AbstractObject<metricType> {
public:

    SimpleObject(
        const Eigen::Vector3<metricType>& initial_position,
        const Eigen::Vector3<metricType>& initial_velocity)
        : AbstractObject<metricType>(
            std::make_unique<SimpleKinematicsSystem<metricType>>(),
            createSimpleInitParams(initial_position, initial_velocity)){}

private:
    static std::unique_ptr<ObjInitParams<metricType>> createSimpleInitParams(
        const Eigen::Vector3<metricType>& position,
        const Eigen::Vector3<metricType>& velocity) {

        auto params = std::make_unique<ObjInitParams<metricType>>();

        params->position = position;
        params->velocity = velocity;

        params->eulerAngles = Eigen::Vector3<metricType>::Zero();
        params->angularVelocity = Eigen::Vector3<metricType>::Zero();
        params->totalForce = Eigen::Vector3<metricType>::Zero();
        params->totalMoment = Eigen::Vector3<metricType>::Zero();
        params->mass = 0.0;
        params->inertia = Eigen::Vector3<metricType>::Zero();

        return params;
    }
};