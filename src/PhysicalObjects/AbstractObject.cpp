//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#include "PCH.h"
#include "AbstractObject.h"

template<typename metricType>
AbstractObject<metricType>::AbstractObject(
    std::unique_ptr<IDynamicsSystem<metricType>> sys,
    std::unique_ptr<ObjInitParams<metricType>> init_params)
    : sys_(std::move(sys))
{
    if (!init_params || !sys_) {
        throw std::invalid_argument("ObjInitParams или DynamicsSystem не может быть null");
    }

    // Создаём кинематику через Builder
    auto kinematics = KinematicState<metricType>::createBuilder()
        .setPosition(init_params->position)
        .setVelocity(init_params->velocity)
        .setEulerAngles(init_params->eulerAngles)
        .setAngularVelocity(init_params->angularVelocity)
        .build();

    presentSnapshot_ = ObjSnapshot<metricType>::createBuilder(kinematics).buildUnique();
    if (!presentSnapshot_) {
        throw std::runtime_error("Failed to create initial snapshot");
    }
}

template<typename metricType>
IDynamicsSystem<metricType>* AbstractObject<metricType>::getDynamicSys() const {
    if (!sys_) {
        throw std::runtime_error("Dynamics system is not initialized");
    }
    return sys_.get();
}

template<typename metricType>
const ObjSnapshot<metricType>& AbstractObject<metricType>::getStateSnapshot() const {
    if (!presentSnapshot_) {
        throw std::runtime_error("Snapshot is not initialized");
    }
    return *presentSnapshot_;
}

template<typename metricType>
void AbstractObject<metricType>::updateSnapshot(std::unique_ptr<ObjSnapshot<metricType>> new_snapshot) {
    if (!new_snapshot) {
        throw std::invalid_argument("New snapshot cannot be null");
    }
    presentSnapshot_ = std::move(new_snapshot);
}

template<typename metricType>
void AbstractObject<metricType>::setActive() {
    currentState_ = ObjectState::ACTIVE;
}

template<typename metricType>
void AbstractObject<metricType>::setDestroyed() {
    currentState_ = ObjectState::DESTROYED;
}

template<typename metricType>
void AbstractObject<metricType>::setCollided() {
    currentState_ = ObjectState::COLLIDED;
}

template<typename metricType>
bool AbstractObject<metricType>::isActive() const noexcept {
    return currentState_ == ObjectState::ACTIVE;
}

template<typename metricType>
bool AbstractObject<metricType>::isCollided() const noexcept {
    return currentState_ == ObjectState::COLLIDED;
}

template<typename metricType>
bool AbstractObject<metricType>::isDestroyed() const noexcept {
    return currentState_ == ObjectState::DESTROYED;
}

// Explicit template instantiation
template class AbstractObject<GLOBAL_CONFIG::PROJECT_TYPE>;