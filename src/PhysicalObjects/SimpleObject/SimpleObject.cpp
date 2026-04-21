#include "SimpleObject.h"
#include "../AbstractObject.h"

template<typename metricType>
SimpleObject<metricType>::SimpleObject(
    const Eigen::Vector3<metricType>& initial_position,
    const Eigen::Vector3<metricType>& initial_velocity)
: AbstractObject<metricType>(
    std::make_unique<SimpleKinematicsSystem<metricType>>(),
    createSimpleInitParams(initial_position, initial_velocity)){}

template<typename metricType>
std::unique_ptr<ObjInitParams<metricType>> SimpleObject<metricType>::createSimpleInitParams(
    const Eigen::Vector3<metricType>& position,
    const Eigen::Vector3<metricType>& velocity
) {
    auto params = std::make_unique<ObjInitParams<metricType>>();
    params->position = position;
    params->velocity = velocity;
    params->eulerAngles = Eigen::Vector3<metricType>::Zero();
    params->angularVelocity = Eigen::Vector3<metricType>::Zero();
    return params;
}


template class SimpleObject<GLOBAL_CONFIG::PROJECT_TYPE>;