/*#pragma once

#include "../../PCH.h"
#include "../../src/utils/SimulationDescriber.h"
#include "../../src/WorldModel/AbstractWorldModel.h"
#include "../../src/OdeSolver/ODESolver.h"
#include "../../src/SimulationMomento/SimulationMomento.h"
#include "../../src/PhysicalObjects/ObjectManager.h"


/**
 * \brief Класс - настройка, в котором происходит выбор вариации задачи, настройка параметров (от параметров мира до ЛА и цели)
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details В общем, тут просто выбирается решатель ОДУ, настраивается сложность окружающего мира (плоской земли до гористой местности).
 *  Я старался разделить задачу максимально, по СОЛИДному. Это позволит бесконечно допиливать программу, добавляя новые модули для бОльшего приближения к реальности.
 *  По факту можно было использовать готовый движок, но как по мне, мне п...й. По факту я хотел подкрепить навыки написания инфраструтуры, которую в дальнейшем можно будет поддерживать.
 #1#

//TODO

template <typename metricType, typename CallbackType>
class IModel {
public:
    IModel(
        std::shared_ptr<ODESolver<metricType, CallbackType>> solver,
        std::shared_ptr<AbstractWorldModel<metricType>> world,
        std::shared_ptr<SimulationMomento<metricType>> dataSaver,
        std::shared_ptr<SimulationDescriber> describer,
        std::shared_ptr<ObjectManager<metricType>> manager,
        CallbackType continue_callback,
        metricType dt);

    IModel(const IModel&) = delete;
    IModel& operator=(const IModel&) = delete;
    IModel(IModel&&) = default;
    IModel& operator=(IModel&&) = default;
    void run();

private:
    std::shared_ptr<ODESolver<metricType, CallbackType>> odeSolver_;
    std::shared_ptr<AbstractWorldModel<metricType>> worldModel_;
    std::shared_ptr<SimulationMomento<metricType>> simulationData_;
    std::shared_ptr<SimulationDescriber> simulationDescriber_;
    std::shared_ptr<ObjectManager<metricType>> obj_manager_;
    CallbackType continue_callback_;
    metricType dt_;
};

#include "../../src/Model.tpp"*/