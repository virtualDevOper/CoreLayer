// src/utils/SimulationConfig.cpp
#include "SimulationConfig.h"
#include <fstream>

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ПАРСИНГА
// ============================================================================

static Eigen::Vector3<double> parseVector3Safe(const nlohmann::json& arr, 
                                               const Eigen::Vector3<double>& default_val) {
    if (!arr.is_array() || arr.size() < 3) return default_val;
    return Eigen::Vector3<double>(
        arr.at(0).get<double>(),
        arr.at(1).get<double>(),
        arr.at(2).get<double>()
    );
}

// ============================================================================
// ПАРСИНГ ВЛОЖЕННЫХ СТРУКТУР
// ============================================================================

Eigen::Vector3<double> SimulationConfig::parseVector3(const nlohmann::json& arr) {
    if (!arr.is_array() || arr.size() != 3) {
        throw std::runtime_error("Vector3 must be array of 3 numbers");
    }
    return Eigen::Vector3<double>(
        arr.at(0).get<double>(),
        arr.at(1).get<double>(),
        arr.at(2).get<double>()
    );
}

DeviceInitialState SimulationConfig::parseInitialState(const nlohmann::json& state) {
    DeviceInitialState init;
    if (state.contains("position")) {
        init.position = parseVector3(state.at("position"));
    }
    if (state.contains("velocity")) {
        init.velocity = parseVector3(state.at("velocity"));
    }
    if (state.contains("euler")) {
        init.euler = parseVector3(state.at("euler"));
    }
    if (state.contains("angular_velocity")) {
        init.angular_velocity = parseVector3(state.at("angular_velocity"));
    }
    return init;
}

DeviceConfig SimulationConfig::parseDevice(const nlohmann::json& dev) {
    DeviceConfig cfg;
    cfg.id = dev.at("id").get<int>();
    cfg.name = dev.at("name").get<std::string>();
    cfg.config_path = dev.at("config_path").get<std::string>();
    if (dev.contains("initial_state")) {
        cfg.initial_state = parseInitialState(dev.at("initial_state"));
    }
    return cfg;
}

StopConditions SimulationConfig::parseStopConditions(const nlohmann::json& stop) {
    StopConditions sc;
    
    if (stop.contains("max_time")) {
        sc.max_time = stop.at("max_time").get<double>();
    }
    
    if (stop.contains("main_object_id")) {
        sc.main_object_id = stop.at("main_object_id").get<int>();
    }
    
    if (stop.contains("min_height")) {
        StopConditions::MinHeight mh;
        const auto& mh_cfg = stop.at("min_height");
        
        if (mh_cfg.is_number()) {
            mh.value = mh_cfg.get<double>();
            mh.object_id = sc.main_object_id;  // по умолчанию — главный объект
        }
        else if (mh_cfg.is_object()) {
            if (mh_cfg.contains("object_id")) {
                mh.object_id = mh_cfg.at("object_id").get<int>();
            }
            mh.value = mh_cfg.at("value").get<double>();
        }
        sc.min_height = mh;
    }
    
    if (stop.contains("logic")) {
        sc.logic = stop.at("logic").get<std::string>();
    }
    
    return sc;
}

// ============================================================================
// ОСНОВНОЙ ПАРСЕР
// ============================================================================

SimulationConfig SimulationConfig::parse(const nlohmann::json& json) {
    SimulationConfig cfg;
    
    // === Глобальные параметры ===
    cfg.operator_name = json.value("operator_name", std::string(""));
    cfg.ode_solver = json.value("ode_solver", std::string("RungeKutta4"));
    cfg.world_config = json.value("world_config", std::string("SimpleFlatEarth"));
    cfg.data_saver = json.value("data_saver", std::string("CSV"));
    cfg.describer = json.value("describer", std::string("Simple Describer"));
    cfg.time_step = json.value("time_step", 0.01);
    cfg.output_csv = json.value("output_csv", std::string("../data/output/results_data/simulation_data.csv"));
    cfg.log_dir = json.value("log_dir", std::string("../logs"));
    cfg.logger_type = json.value("logger_type", std::string("both"));
    
    // === Устройства ===
    if (json.contains("devices") && json.at("devices").is_array()) {
        for (const auto& dev : json.at("devices")) {
            cfg.devices.push_back(parseDevice(dev));
        }
    }
    
    // === Условия остановки ===
    if (json.contains("stop_conditions")) {
        cfg.stop_conditions = parseStopConditions(json.at("stop_conditions"));
    }
    
    return cfg;
}

SimulationConfig SimulationConfig::loadFromJsonFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + path);
    }
    
    nlohmann::json json;
    file >> json;
    
    return parse(json);
}