/*
//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../DynamicsSystem/IDynamicsSystem.h"
#include "../DynamicsSystem/ExtensionModels/Aerodinamics/AeroInput/AeroInput.h"
#include "../utils/ObjSnapshot.h"
#include "../utils/ObjInitParams.h"
/**
 * \brief Абстрактный класс объекта динамической системы
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Это просто класс состояния объекта в определенный момент времени, у него есть состояние,которое меняется каждый шаг решения системы ОДУ. В свою очередб параметры, такие как масса, момент инерции и тд будут меняться
 *  В свою очередь писать дополнительную модель для вычисления всего - просто п....ц какая огромная задача  (если так сделать, то в КБ нужен будет один оператор данной программы, а остальных в утиль).
 *  ПОЭТОМУ некоторые параметры будут просто задаваться табличными значениями по времени, которые потом будут интерполироваться.
 *
 #1#

//TODO
// === Нужно сделать реальные параметры и параметры, которые были рассчитаны на внутреннем вычислителе с датчиков  ===



template<typename metricType>
class AbstractObject {
public:
    // Принимаем shared_ptr по значению для возможности перемещения
    explicit AbstractObject(std::shared_ptr<IDynamicsSystem<metricType>> sys,
                            ObjInitParams<metricType> init_params) // Принимаем по значению
        : sys_(std::move(sys)) // Перемещаем sys
    {
        presentSnapshot_.setTotalForce(std::move(init_params.totalForce));
        presentSnapshot_.setTotalMoment(std::move(init_params.totalMoment));
        presentSnapshot_.setVelocity(std::move(init_params.velocity));
        presentSnapshot_.setAcceleration(std::move(init_params.acceleration));
        presentSnapshot_.setAngularVelocity(std::move(init_params.angularVelocity));
        presentSnapshot_.setAngularAcceleration(std::move(init_params.angularAcceleration));
        presentSnapshot_.setMass(std::move(init_params.mass));
        presentSnapshot_.setInertia(std::move(init_params.inertia));
        presentSnapshot_.setCenterOfMass(std::move(init_params.centerOfMass));
        presentSnapshot_.setPosition(std::move(init_params.position));
        presentSnapshot_.setEulerAngles(std::move(init_params.eulerAngles));
    };

    virtual ~AbstractObject() = default;

    const ObjSnapshot<metricType>& getStateSnapshot() const { return presentSnapshot_; }

    void updateSnapshot(ObjSnapshot<metricType>&& new_snapshot) {
        presentSnapshot_ = std::move(new_snapshot);
    }

    const std::shared_ptr<IDynamicsSystem<metricType>>& getDynamicSys() const { return sys_; }

    enum class ObjectState {
        ACTIVE,
        DESTROYED,
        COLLIDED
    };

    void setActive() { currentState_ = ObjectState::ACTIVE; }
    void setDestroyed() { currentState_ = ObjectState::DESTROYED; }
    void setCollided() { currentState_ = ObjectState::COLLIDED; }

    bool isActive() const { return currentState_ == ObjectState::ACTIVE; }
    bool isCollided() const { return currentState_ == ObjectState::COLLIDED; }
    bool isDestroyed() const { return currentState_ == ObjectState:: DESTROYED; }

    //TODO
    // === ОРИЕНТАЦИЯ В КВАТЕРНИОНАХ (ДОПОЛНИТЕЛЬНЫЙ МЕТОД) ===
    // === МЕТОДЫ ПРЕОБРАЗОВАНИЯ МЕЖДУ УГЛАМИ И КВАТЕРНИОНАМИ ===
    // === ГЕОМЕТРИЯ ОБЪЕКТА ===
    // === СЕТЧАТАЯ МОДЕЛЬ ДЛЯ ВИЗУАЛИЗАЦИИ ===

protected:
    ObjSnapshot<metricType> presentSnapshot_;
    std::shared_ptr<IDynamicsSystem<metricType>> sys_;
    std::shared_ptr<AeroInput<metricType>> aerodynamicParams_;
    ObjectState currentState_ = ObjectState::ACTIVE;
};
*/

