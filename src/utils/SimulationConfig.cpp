#include "SimulationConfig.h"

#include <fstream>
#include <sstream>



namespace {
    std::string findKeyPattern(const std::string& key) {
        return "\"" + key + "\"";
    }

    std::string::size_type findValueStart(const std::string& json, std::string::size_type pos) {
        auto colon = json.find(':', pos);
        if (colon == std::string::npos) {
            throw std::runtime_error("SimulationConfig: ':' not found after key");
        }
        auto value_start = json.find_first_not_of(" \t\n\r", colon + 1);
        if (value_start == std::string::npos) {
            throw std::runtime_error("SimulationConfig: value not found after ':'");
        }
        return value_start;
    }
}

SimulationConfig SimulationConfig::loadFromJsonFile(const std::string& path) {
    SimulationConfig cfg;
    const auto json = readFileToString(path);

    // Глобальные параметры симуляции
    cfg.time_step = extractFloat(json, "time_step", cfg.time_step);
    cfg.max_time = extractFloat(json, "max_time", cfg.max_time);
    cfg.output_csv = extractString(json, "output_csv", cfg.output_csv);

    // Новые строки для описания
    cfg.operator_name = extractString(json, "operator_name", cfg.operator_name);
    cfg.ode_solver = extractString(json, "ode_solver", cfg.ode_solver);
    cfg.world_config = extractString(json, "world_config", cfg.world_config);
    cfg.data_saver = extractString(json, "data_saver", cfg.data_saver);
    cfg.earth_type = extractString(json, "earth_type", cfg.earth_type);

    // Пути к таблицам
    cfg.thrust_x_path = extractString(json, "thrust_x", cfg.thrust_x_path);
    cfg.thrust_y_path = extractString(json, "thrust_y", cfg.thrust_y_path);
    cfg.thrust_z_path = extractString(json, "thrust_z", cfg.thrust_z_path);
    cfg.mass_path = extractString(json, "mass", cfg.mass_path);
    cfg.Ixx_path = extractString(json, "Ixx", cfg.Ixx_path);
    cfg.Iyy_path = extractString(json, "Iyy", cfg.Iyy_path);
    cfg.Izz_path = extractString(json, "Izz", cfg.Izz_path);

    // Начальные условия
    cfg.rocket_init.position = extractVec3(json, "position", cfg.rocket_init.position);
    cfg.rocket_init.velocity = extractVec3(json, "velocity", cfg.rocket_init.velocity);
    cfg.rocket_init.euler = extractVec3(json, "euler", cfg.rocket_init.euler);
    cfg.rocket_init.angular_velocity = extractVec3(json, "angular_velocity", cfg.rocket_init.angular_velocity);

    return cfg;
}

std::string SimulationConfig::readFileToString(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("SimulationConfig: cannot open config file: " + path);
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

float SimulationConfig::extractFloat(const std::string& json, const std::string& key, float default_value) {
    const auto pattern = findKeyPattern(key);
    auto pos = json.find(pattern);
    if (pos == std::string::npos) {
        return default_value;
    }

    auto value_start = findValueStart(json, pos + pattern.size());

    // Читаем до конца числа
    auto value_end = json.find_first_of(",}\n\r", value_start);
    if (value_end == std::string::npos) {
        value_end = json.size();
    }
    auto token = json.substr(value_start, value_end - value_start);
    try {
        return std::stof(token);
    } catch (const std::exception&) {
        return default_value;
    }
}

std::string SimulationConfig::extractString(const std::string& json, const std::string& key, const std::string& default_value) {
    const auto pattern = findKeyPattern(key);
    auto pos = json.find(pattern);
    if (pos == std::string::npos) {
        return default_value;
    }

    auto value_start = findValueStart(json, pos + pattern.size());
    if (json[value_start] != '"') {
        return default_value;
    }
    ++value_start;
    auto value_end = json.find('"', value_start);
    if (value_end == std::string::npos) {
        throw std::runtime_error("SimulationConfig: unterminated string for key: " + key);
    }
    return json.substr(value_start, value_end - value_start);
}

Eigen::Vector3<float> SimulationConfig::extractVec3(const std::string& json, const std::string& key, const Eigen::Vector3<float>& default_value) {
    const auto pattern = findKeyPattern(key);
    auto pos = json.find(pattern);
    if (pos == std::string::npos) {
        return default_value;
    }

    auto value_start = findValueStart(json, pos + pattern.size());
    if (json[value_start] != '[') {
        return default_value;
    }
    ++value_start;
    auto value_end = json.find(']', value_start);
    if (value_end == std::string::npos) {
        throw std::runtime_error("SimulationConfig: unterminated array for key: " + key);
    }

    auto array_content = json.substr(value_start, value_end - value_start);
    std::istringstream ss(array_content);
    std::string part;
    float values[3] = { default_value.x(), default_value.y(), default_value.z() };
    int idx = 0;
    while (std::getline(ss, part, ',') && idx < 3) {
        try {
            values[idx] = std::stof(part);
        } catch (const std::exception&) {
            // оставляем значение по умолчанию
        }
        ++idx;
    }
    return Eigen::Vector3<float>(values[0], values[1], values[2]);
}
