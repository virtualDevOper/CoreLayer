//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../DynamicsSystem/IDynamicsSystem.h"
#include "../utils/ObjSnapshot.h"
#include "../utils/ObjInitParams.h"
#include "../utils/Interpolation/ComponentInterpolationManager.h"

/**
 * \brief Базовый класс для всех физических объектов в симуляции.
 *
 * \tparam metricType Тип данных для метрических величин.
 *
 * \details Абстрактный класс, представляющий физический объект с состоянием
 * и системой динамики. Хранит текущий снимок состояния (кинематика + параметры)
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
        std::unique_ptr<ObjInitParams<metricType>> init_params);

    virtual ~AbstractObject() = default;

    // === ДОСТУП К СИСТЕМЕ ОДУ ===
    // Возвращаем сырой указатель для наблюдения (не передаем владение)
    IDynamicsSystem<metricType>* getDynamicSys() const;

    // === ДОСТУП К СОСТОЯНИЮ ===
    const ObjSnapshot<metricType>& getStateSnapshot() const;
    void updateSnapshot(std::unique_ptr<ObjSnapshot<metricType>> new_snapshot);

    // === СТАТУС ОБЪЕКТА ===
    enum class ObjectState { ACTIVE, DESTROYED, COLLIDED };
    void setActive();
    void setDestroyed();
    void setCollided();
    [[nodiscard]] bool isActive() const noexcept;
    [[nodiscard]] bool isCollided() const noexcept;
    [[nodiscard]] bool isDestroyed() const noexcept;
    
    AbstractObject(const AbstractObject&) = delete;
    AbstractObject& operator=(const AbstractObject&) = delete;
protected:
    std::unique_ptr<ObjSnapshot<metricType>> presentSnapshot_;
    std::unique_ptr<IDynamicsSystem<metricType>> sys_;  // Объект владеет своей системой ОДУ
    ObjectState currentState_ = ObjectState::ACTIVE;
};
