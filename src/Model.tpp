//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once

template <typename metricType, typename CallbackType>
IModel<metricType, CallbackType>::IModel(
    std::unique_ptr<ODESolver<metricType, CallbackType>> solver,
    std::shared_ptr<AbstractWorldModel<metricType>> world,
    std::unique_ptr<SimulationMomento<metricType>> dataSaver,
    std::unique_ptr<SimulationDescriber> describer,
    std::shared_ptr<IObjectManager<metricType>> manager,
    CallbackType continue_callback,
    metricType dt)

    : odeSolver_(std::move(solver)),
      worldModel_(std::move(world)),
      simulationData_(std::move(dataSaver)),
      simulationDescriber_(std::move(describer)),
      obj_manager_(std::move(manager)),
      continue_callback_(std::move(continue_callback)),
      dt_(dt)
{
    if (!odeSolver_) { throw std::invalid_argument("IModel: solver is null"); }
    if (!worldModel_) { throw std::invalid_argument("IModel: world model is null"); }
    if (!obj_manager_) { throw std::invalid_argument("IModel: object manager is null"); }
    if (!simulationData_) { throw std::invalid_argument("IModel: data saver is null"); }
};

template <typename metricType, typename CallbackType>
void IModel<metricType, CallbackType>::run() {
    try {
        if (simulationDescriber_) {
            std::cout << "start time: " << simulationDescriber_->start_time << std::endl;
            std::cout << "operator name: " << simulationDescriber_->operator_name << std::endl;
            std::cout << "ode solver: " << simulationDescriber_->ode_solver << std::endl;
            std::cout << "world config: " << simulationDescriber_->world_config << std::endl;
            std::cout << "data saver: " << simulationDescriber_->data_saver << std::endl;
            std::cout << "earth type: " << simulationDescriber_->earth_type << std::endl;
            std::cout << "simulation objects: ";
            for (const auto& obj : simulationDescriber_->simulation_objects) {
                std::cout << obj << " ";
            }
            std::cout << std::endl;
        }
            if (!simulationData_) {
                throw std::runtime_error("Simulation data saver was nullified after construction");
            }
                this->odeSolver_->solve(
            obj_manager_,
            0,
            dt_,
            continue_callback_,
            *simulationData_
            );

            simulationData_->save();

    } catch (const std::exception& e) {
        std::cerr << "Ошибка в симуляции: " << e.what() << std::endl;
        throw;
    }
}