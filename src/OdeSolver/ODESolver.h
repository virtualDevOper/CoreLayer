//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "PCH.h"
#include "../utils/ObjManager/IObjectManager.h"
#include "../SimulationMomento/SimulationMomento.h"

/**
 * \brief Абстрактный интерфейс для решателей обыкновенных дифференциальных уравнений.
 *
 * \tparam metricType Тип данных для метрических величин.
 * \tparam CallbackType Тип функции обратного вызова для контроля симуляции.
 *
 * \details Определяет контракт для всех численных интеграторов в симуляторе.
 * Решает системы ОДУ для множества объектов одновременно, управляет
 * временным шагом и сохраняет историю состояний. Поддерживает проверку
 * коллизий и условий остановки через callback-функции.
 */

//TODO
// === Реализовать проверку коллизий между объектами и с землей ===

template <typename metricType, typename CallbackType>
class ODESolver {
public:
    virtual ~ODESolver() = default;

    virtual void solve(
        std::shared_ptr<IObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        SimulationMomento<metricType>& momento
    ) = 0;

protected:
    std::vector<int> checkCollisions(const std::vector<StateStorage<metricType>>& stateStorages) {
        // TODO: Реализовать проверку коллизий
        return {};
    }
};