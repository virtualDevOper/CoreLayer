#pragma once
#include "PCH.h"

/**
 * \brief Минимальная JSON-конфигурация запуска симуляции.
 *
 * Парсер предельно простой и ориентирован только на ожидаемую структуру.
 * Это не общий JSON-парсер, а специализированный конфиг-ридер.
 */

struct SimulationConfigRocketInit {
    Eigen::Vector3<float> position{0.f, 0.f, 0.f};
    Eigen::Vector3<float> velocity{0.f, 0.f, 0.f};
    Eigen::Vector3<float> euler{0.f, 0.f, 0.f};
    Eigen::Vector3<float> angular_velocity{0.f, 0.f, 0.f};
};

struct SimulationConfig {
    std::string operator_name;
    std::string ode_solver;
    std::string world_config;
    std::string data_saver;
    std::string earth_type;

    float time_step;  // Разумное значение по умолчанию
    float max_time;    // Минимальное разумное значение
    std::string output_csv;

    // Пути к табличным данным
    std::string thrust_x_path;
    std::string thrust_y_path;
    std::string thrust_z_path;
    std::string mass_path;
    std::string Ixx_path;
    std::string Iyy_path;
    std::string Izz_path;
    std::string COM_x_path;
    std::string COM_y_path;
    std::string COM_z_path;

    SimulationConfigRocketInit rocket_init;

    static SimulationConfig loadFromJsonFile(const std::string& path);

private:
    static std::string readFileToString(const std::string& path);
    static float extractFloat(const std::string& json, const std::string& key, float default_value);
    static std::string extractString(const std::string& json, const std::string& key, const std::string& default_value);
    static Eigen::Vector3<float> extractVec3(const std::string& json, const std::string& key, const Eigen::Vector3<float>& default_value);
};