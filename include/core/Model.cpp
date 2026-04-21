#include "PCH.h"

#include "Model.h"

inline std::string formatUnixTime(const uint64_t unix_time) {
    const auto time_t_value = static_cast<std::time_t>(unix_time);
    
    std::tm tm_utc{};
#ifdef _WIN32
    if (gmtime_s(&tm_utc, &time_t_value) != 0) {
        throw std::runtime_error("Failed to convert time");
    }
#else
    if (gmtime_r(&time_t_value, &tm_utc) == nullptr) {
        throw std::runtime_error("Failed to convert time");
    }
#endif

    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << tm_utc.tm_mday << "/"
        << std::setw(2) << (tm_utc.tm_mon + 1) << "/"
        << (tm_utc.tm_year + 1900) << " "
        << std::setw(2) << tm_utc.tm_hour << ":"
        << std::setw(2) << tm_utc.tm_min << ":"
        << std::setw(2) << tm_utc.tm_sec;
    return oss.str();
}

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
    if (!odeSolver_) { 
        throw std::invalid_argument("IModel: solver is null"); 
    }
    if (!worldModel_) { 
        throw std::invalid_argument("IModel: world model is null"); 
    }
    if (!obj_manager_) { 
        throw std::invalid_argument("IModel: object manager is null"); 
    }
    if (!simulationData_) { 
        throw std::invalid_argument("IModel: data saver is null"); 
    }
}

template <typename metricType, typename CallbackType>
void IModel<metricType, CallbackType>::run() {
    try {
        if (simulationDescriber_) {
            Logger::getInstance().info("start time: " + formatUnixTime(simulationDescriber_->start_time));
            Logger::getInstance().info("operator name: " + simulationDescriber_->operator_name);
            Logger::getInstance().info("ode solver: " + simulationDescriber_->ode_solver);
            Logger::getInstance().info("world config: " + simulationDescriber_->world_config);
            std::string LAs;
            for (const auto& obj : simulationDescriber_->simulation_objects) {LAs += obj+ "; ";}
            Logger::getInstance().info("simulation objects: " + LAs);
        }
        
        odeSolver_->solve(obj_manager_, 0, dt_, continue_callback_, *simulationData_);
        simulationData_->save();

    } catch (const std::exception& e) {
        Logger::getInstance().error("Simulation error: " + std::string(e.what()));
        throw;
    }
}

#include "GLOBAL_CONFIG.h"
template class IModel<GLOBAL_CONFIG::PROJECT_TYPE, GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>>;