/*
//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once

template <typename metricType, typename CallbackType>
IModel<metricType, CallbackType>::IModel(
    std::shared_ptr<ODESolver<metricType,CallbackType>> solver,
    std::shared_ptr<AbstractWorldModel<metricType>> world,
    std::shared_ptr<SimulationMomento<metricType>> dataSaver,
    std::shared_ptr<SimulationDescriber> describer,
    std::shared_ptr<ObjectManager<metricType>> manager,
    CallbackType continue_callback,
    metricType dt)

    : odeSolver_(std::move(solver)),
      worldModel_(std::move(world)),
      simulationData_(std::move(dataSaver)),
      simulationDescriber_(std::move(describer)),
      obj_manager_(std::move(manager)),
      continue_callback_(continue_callback),
      dt_(dt)
{};

template <typename metricType, typename CallbackType>
void IModel<metricType, CallbackType>::run() {
    try {this->odeSolver_->solve(
        obj_manager_, 0, dt_, continue_callback_,simulationData_
        );
    } catch (const std::exception& e) {std::cerr << "Ошибка: " << e.what() << std::endl;}
}
*/



