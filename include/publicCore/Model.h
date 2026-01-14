#pragma once

#include "../../PCH.h"
#include "../../src/utils/SimulationDescriber.h"
#include "../../src/WorldModel/AbstractWorldModel.h"
#include "../../src/OdeSolver/ODESolver.h"
#include "../../src/SimulationMomento/SimulationMomento.h"
#include "../../src/PhysicalObjects/ObjectManager.h"

/**
 * \brief Класс модели симуляции, управляющий решателем ОДУ, миром и объектами
 *
 * \tparam metricType Тип данных для метрических величин
 * \tparam CallbackType Тип функции обратного вызова для остановки симуляции
 *
 * \details Класс объединяет все компоненты симуляции: решатель ОДУ, модель мира,
 * менеджер объектов и сохранение данных. Обеспечивает управление жизненным циклом
 * всех компонентов и запуск симуляции.
 */

template <typename metricType, typename CallbackType>
class IModel {
public:
    IModel(
        std::unique_ptr<ODESolver<metricType, CallbackType>> solver,
        std::shared_ptr<AbstractWorldModel<metricType>> world,
        std::unique_ptr<SimulationMomento<metricType>> dataSaver,
        std::unique_ptr<SimulationDescriber> describer,
        std::shared_ptr<ObjectManager<metricType>> manager,
        CallbackType continue_callback,
        metricType dt);

    IModel(const IModel&) = delete;
    IModel& operator=(const IModel&) = delete;
    IModel(IModel&&) = default;
    IModel& operator=(IModel&&) = default;

    void run();

private:
    std::unique_ptr<ODESolver<metricType, CallbackType>> odeSolver_;
    std::shared_ptr<AbstractWorldModel<metricType>> worldModel_;
    std::unique_ptr<SimulationMomento<metricType>> simulationData_;
    std::unique_ptr<SimulationDescriber> simulationDescriber_;
    std::shared_ptr<ObjectManager<metricType>> obj_manager_;
    CallbackType continue_callback_;
    metricType dt_;
};

#include "../../src/Model.tpp"