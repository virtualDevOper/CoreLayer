// src/utils/SimulationConfig.h
#pragma once
#include "PCH.h"

// ============================================================================
// СТРУКТУРЫ ДЛЯ НОВОГО ФОРМАТА
// ============================================================================

struct DeviceInitialState {
    Eigen::Vector3<double> position{0.0, 0.0, 0.0};
    Eigen::Vector3<double> velocity{0.0, 0.0, 0.0};
    Eigen::Vector3<double> euler{0.0, 0.0, 0.0};
    Eigen::Vector3<double> angular_velocity{0.0, 0.0, 0.0};
};

struct DeviceConfig {
    int id;
    std::string name;
    std::string config_path;
    DeviceInitialState initial_state;
};

struct StopConditions {
    std::optional<double> max_time;
    std::optional<int> main_object_id;

    struct MinHeight {
        std::optional<int> object_id;
        double value;
    };
    std::optional<MinHeight> min_height;

    std::string logic = "OR";  // "AND" или "OR"
};

// ============================================================================
// ОСНОВНОЙ КЛАСС КОНФИГУРАЦИИ
// ============================================================================

struct SimulationConfig {
    // === Глобальные параметры ===
    std::string operator_name;
    std::string ode_solver;
    std::string world_config;
    std::string data_saver;
    std::string describer;
    double time_step;
    std::string output_csv;
    std::string log_dir;
    std::string logger_type;

    // === Устройства (массив) ===
    std::vector<DeviceConfig> devices;

    // === Условия остановки ===
    StopConditions stop_conditions;

    // === Статические методы загрузки ===
    static SimulationConfig loadFromJsonFile(const std::string& path);
    static SimulationConfig parse(const nlohmann::json& json);

private:
    static DeviceConfig parseDevice(const nlohmann::json& dev);
    static StopConditions parseStopConditions(const nlohmann::json& stop);
    static DeviceInitialState parseInitialState(const nlohmann::json& state);
    static Eigen::Vector3<double> parseVector3(const nlohmann::json& arr);
};