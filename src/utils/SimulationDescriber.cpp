#include "PCH.h"
#include "SimulationDescriber.h"



void SimulationDescriber::setOperatorName(const std::string& name) {
    operator_name = name;
}

void  SimulationDescriber::set_offcet(const int utc_offset) {
    utc_offset_ = utc_offset;
    start_time = std::chrono::system_clock::to_time_t(
    std::chrono::system_clock::now() + std::chrono::hours(utc_offset));
}

void SimulationDescriber::setOdeSolver(const std::string& solver) {
    ode_solver = solver;
}

void SimulationDescriber::setWorldConfig(const std::string& config) {
    world_config = config;
}

void SimulationDescriber::setDataSaver(const std::string& saver) {
    data_saver = saver;
}

void SimulationDescriber::setEarthType(const std::string& type) {
    earth_type = type;
}

void SimulationDescriber::setSimulationObjects(const std::vector<int>& ids) {
    simulation_objects.reserve(ids.size());
    for (int id : ids) {
        simulation_objects.push_back("ID: " + std::to_string(id));
    }
}