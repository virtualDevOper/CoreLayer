//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../DynamicsSystem/IDynamicsSystem.h"
#include "../utils/ObjSnapshot.h"
#include "../utils/ObjInitParams.h"
#include "../utils/Interpolation/ComponentInterpolationManager.h"
/**
 * \brief Абстрактный класс объекта динамической системы
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Класс состояния объекта в определенный момент времени, у него есть состояние,которое меняется каждый шаг решения системы ОДУ. В свою очередб параметры, такие как масса, момент инерции и тд будут меняться
 *  В свою очередь писать дополнительную модель для вычисления всего - просто п....ц какая огромная задача  (если так сделать, то в КБ нужен будет один оператор данной программы, а остальных в утиль).
 *  ПОЭТОМУ некоторые параметры будут просто задаваться табличными значениями по времени, которые потом будут интерполироваться, но это в наследниках+_)))).
 *
 */

//TODO
// === Нужно сделать реальные параметры и параметры, которые были рассчитаны на внутреннем вычислителе с датчиков  ===


template<typename metricType>
class AbstractObject {
public:
    explicit AbstractObject(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> init_params)
        : sys_(std::move(sys)) {

        if (!init_params || !sys_) {
            throw std::invalid_argument("ObjInitParams или DynamicsSystem не может быть null");
        }

        presentSnapshot_ = ObjSnapshot<metricType>::createBuilder()
            .setPosition(init_params->position)
            .setVelocity(init_params->velocity)
            .setEulerAngles(init_params->eulerAngles)
            .setAngularVelocity(init_params->angularVelocity)
            .buildUnique();

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
    std::shared_ptr<IDynamicsSystem<metricType>> sys_; // Изменено на shared_ptr для безопасного доступа
    ObjectState currentState_ = ObjectState::ACTIVE;

    // Запрещаем копирование
    AbstractObject(const AbstractObject&) = delete;
    AbstractObject& operator=(const AbstractObject&) = delete;
};

