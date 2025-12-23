//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../DynamicsSystem/IDynamicsSystem.h"
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
 */

//TODO
// === Нужно сделать реальные параметры и параметры, которые были рассчитаны на внутреннем вычислителе с датчиков  ===



//переписать прийдется на простой и компелксный объект

template<typename metricType>
class AbstractObject {
public:
    explicit AbstractObject(std::unique_ptr<IDynamicsSystem<metricType>> sys,
                           std::unique_ptr<ObjInitParams<metricType>> init_params,
                           std::unique_ptr<ObjInitParams<metricType>> init_params,

                           )
       : sys_(std::move(sys))
    {
        auto builder = ObjSnapshot<metricType>::createBuilder();
        builder.setPosition(init_params->position)
            .setEulerAngles(init_params->eulerAngles)
            .setVelocity(init_params->velocity)
            .setAngularVelocity(init_params->angularVelocity)

        // эта фигня будет рассчитываться
            .setTotalMoment(init_params->totalMoment)

            .setTotalForce(init_params->totalForce)
            .setMass(init_params->mass)
            .setInertia(init_params->inertia);

        presentSnapshot_ = builder.buildUnique();
    };


    virtual ~AbstractObject() = default;

    std::shared_ptr<IDynamicsSystem<metricType>> getDynamicSys() const { return sys_; }
    const ObjSnapshot<metricType>& getStateSnapshot() const { return *presentSnapshot_; }

    void updateSnapshot(std::unique_ptr<ObjSnapshot<metricType>> new_snapshot) {
        presentSnapshot_ = std::move(new_snapshot);
    }

    enum class ObjectState { ACTIVE, DESTROYED, COLLIDED };

    void setActive() { currentState_ = ObjectState::ACTIVE; }
    void setDestroyed() { currentState_ = ObjectState::DESTROYED; }
    void setCollided() { currentState_ = ObjectState::COLLIDED; }

    [[nodiscard]] bool isActive() const { return currentState_ == ObjectState::ACTIVE; }
    [[nodiscard]] bool isCollided() const { return currentState_ == ObjectState::COLLIDED; }
    [[nodiscard]] bool isDestroyed() const { return currentState_ == ObjectState::DESTROYED; }

protected:
    std::unique_ptr<ObjSnapshot<metricType>> presentSnapshot_;
    std::shared_ptr<IDynamicsSystem<metricType>> sys_;
    ObjectState currentState_ = ObjectState::ACTIVE;

    тут интерполируемый параметры будут храниться
    //тут геометрия объекта и его модель для визуализации
};

