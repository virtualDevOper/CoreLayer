//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../DynamicsSystem/IDynamicsSystem.h"
#include "../utils/ObjSnapshot.h"
#include "../utils/ObjInitParams.h"
#include "../utils/Interpolation/ComponentInterpolationManager.h"

/**
 * \brief Базовый класс объекта динамической системы.
 *
 * \tparam metricType Тип данных для метрических величин.
 *
 * \details Хранит текущий снимок состояния (кинематика + параметры)
 * и ссылку на систему ОДУ, которая описывает эволюцию этого объекта.
 * Управляет жизненным циклом (ACTIVE/DESTROYED/COLLIDED) и предоставляет
 * унифицированный интерфейс для менеджера объектов.
 */

//TODO
// === Нужно сделать реальные параметры и параметры, которые были рассчитаны на внутреннем вычислителе с датчиков  ===

template<typename metricType>
class AbstractObject {
public:
    explicit AbstractObject(
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

    virtual ~AbstractObject() = default;

    // === ДОСТУП К СИСТЕМЕ ОДУ ===
    std::shared_ptr<IDynamicsSystem<metricType>> getDynamicSys() const {
        if (!sys_) {
            throw std::runtime_error("Dynamics system is not initialized");
        }
        return sys_;
    }

    // === ДОСТУП К СОСТОЯНИЮ ===
    const ObjSnapshot<metricType>& getStateSnapshot() const {
        if (!presentSnapshot_) {
            throw std::runtime_error("Snapshot is not initialized");
        }
        return *presentSnapshot_;
    }

    void updateSnapshot(std::unique_ptr<ObjSnapshot<metricType>> new_snapshot) {
        if (!new_snapshot) {
            throw std::invalid_argument("New snapshot cannot be null");
        }
        presentSnapshot_ = std::move(new_snapshot);
    }

    // === СТАТУС ОБЪЕКТА ===
    enum class ObjectState { ACTIVE, DESTROYED, COLLIDED };
    void setActive() { currentState_ = ObjectState::ACTIVE; }
    void setDestroyed() { currentState_ = ObjectState::DESTROYED; }
    void setCollided() { currentState_ = ObjectState::COLLIDED; }
    [[nodiscard]] bool isActive() const noexcept { return currentState_ == ObjectState::ACTIVE; }
    [[nodiscard]] bool isCollided() const noexcept { return currentState_ == ObjectState::COLLIDED; }
    [[nodiscard]] bool isDestroyed() const noexcept { return currentState_ == ObjectState::DESTROYED; }

protected:
    std::unique_ptr<ObjSnapshot<metricType>> presentSnapshot_;
    std::shared_ptr<IDynamicsSystem<metricType>> sys_;
    ObjectState currentState_ = ObjectState::ACTIVE;

    AbstractObject(const AbstractObject&) = delete;
    AbstractObject& operator=(const AbstractObject&) = delete;
};
