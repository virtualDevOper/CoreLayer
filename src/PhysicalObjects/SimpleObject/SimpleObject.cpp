// src/PhysicalObjects/SimpleObject/SimpleObject.cpp
#include "SimpleObject.h"
#include "../AbstractObject.h"

template<typename metricType>
SimpleObject<metricType>::SimpleObject(
    const Eigen::Vector3<metricType>& initial_position,
    const Eigen::Vector3<metricType>& initial_velocity,
    metricType mass
)
: AbstractObject<metricType>(
    std::make_unique<SimpleKinematicsSystem<metricType>>(),
    createSimpleInitParams(initial_position, initial_velocity))
{
    if (mass > metricType(0)) {
        // Можно добавить массу в snapshot позже если нужно
    }
}

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

template<typename metricType>
void SimpleObject<metricType>::setCollisionThreshold(metricType distance) {
    collision_threshold_ = distance;
}

template<typename metricType>
std::optional<int> SimpleObject<metricType>::checkCollision(const AbstractObject<metricType>* other, int other_id) {
    if (!other || !collision_threshold_) return std::nullopt;

    const auto& this_state = this->getStateSnapshot();
    const auto& other_state = other->getStateSnapshot();

    const auto dist = (this_state.getPosition() - other_state.getPosition()).norm();
    if (dist <= collision_threshold_) {
        collision_detected_ = true;
        return other_id;  // Коллизия с указанным объектом
    }
    return std::nullopt;
}

template<typename metricType>
bool SimpleObject<metricType>::hasCollisionBeenDetected() const noexcept {
    return collision_detected_;
}

// Явные инстанциации
template class SimpleObject<GLOBAL_CONFIG::PROJECT_TYPE>;