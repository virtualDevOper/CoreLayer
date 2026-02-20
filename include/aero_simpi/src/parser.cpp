#include "parser.h"
#include <fstream>
#include <sstream>

// nlohmann/json должен быть доступен в путях компиляции
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace aero {

//==============================================================================
// JSON CONFIG LOADER
//==============================================================================

JsonConfigLoader::JsonConfigLoader(const std::string& filepath) : filepath_(filepath) {}

AeroConfig JsonConfigLoader::load() const {
    return JsonParser::parseFile(filepath_);
}

//==============================================================================
// JSON PARSER
//==============================================================================

AeroConfig JsonParser::parse(const std::string& json_content) {
    AeroConfig config;
    
    try {
        const json j = json::parse(json_content);
        
        // Основная информация
        if (j.contains("aircraft")) {
            const auto& aircraft = j["aircraft"];
            
            if (aircraft.contains("name")) {
                config.name = aircraft["name"].get<std::string>();
            }
            
            // Глобальные параметры
            if (aircraft.contains("S_ref_global")) {
                config.global.S_ref = aircraft["S_ref_global"].get<double>();
            }
            if (aircraft.contains("c_ref_global")) {
                config.global.c_ref = aircraft["c_ref_global"].get<double>();
            }
            if (aircraft.contains("b_ref_global")) {
                config.global.b_ref = aircraft["b_ref_global"].get<double>();
            }
            
            // Центр масс
            if (aircraft.contains("center_of_mass")) {
                const auto& com = aircraft["center_of_mass"];
                if (com.is_array() && com.size() >= 3) {
                    config.x_com = com[0].get<double>();
                    config.y_com = com[1].get<double>();
                    config.z_com = com[2].get<double>();
                }
            }
            
            // Компоненты
            if (aircraft.contains("components")) {
                for (const auto& comp_json : aircraft["components"]) {
                    ComponentConfig comp;
                    
                    if (comp_json.contains("name")) {
                        comp.name = comp_json["name"].get<std::string>();
                    }
                    if (comp_json.contains("type")) {
                        comp.type = parseComponentType(comp_json["type"].get<std::string>());
                    }
                    
                    // Геометрия
                    if (comp_json.contains("length")) {
                        comp.length = comp_json["length"].get<double>();
                    }
                    if (comp_json.contains("diameter")) {
                        comp.diameter = comp_json["diameter"].get<double>();
                    }
                    if (comp_json.contains("S_ref")) {
                        comp.S_ref = comp_json["S_ref"].get<double>();
                    }
                    if (comp_json.contains("b_ref")) {
                        comp.b_ref = comp_json["b_ref"].get<double>();
                    }
                    if (comp_json.contains("c_ref")) {
                        comp.c_ref = comp_json["c_ref"].get<double>();
                    }
                    if (comp_json.contains("AR")) {
                        comp.AR = comp_json["AR"].get<double>();
                    }
                    
                    // Форма носа
                    if (comp_json.contains("nose_type")) {
                        comp.nose_type = parseNoseType(comp_json["nose_type"].get<std::string>());
                    }
                    
                    // Позиция
                    if (comp_json.contains("x_pos")) {
                        comp.x_pos = comp_json["x_pos"].get<double>();
                    }
                    if (comp_json.contains("y_pos")) {
                        comp.y_pos = comp_json["y_pos"].get<double>();
                    }
                    if (comp_json.contains("z_pos")) {
                        comp.z_pos = comp_json["z_pos"].get<double>();
                    }
                    
                    // Рули
                    if (comp_json.contains("can_deflect")) {
                        comp.can_deflect = comp_json["can_deflect"].get<bool>();
                    }
                    if (comp_json.contains("delta")) {
                        comp.delta = comp_json["delta"].get<double>();
                    }
                    if (comp_json.contains("mount_angle")) {
                        comp.mount_angle = comp_json["mount_angle"].get<double>();
                    }
                    
                    // Настраиваемые параметры
                    if (comp_json.contains("k_interference")) {
                        comp.k_interference = comp_json["k_interference"].get<double>();
                    }
                    if (comp_json.contains("wake_shadow_factor")) {
                        comp.wake_shadow_factor = comp_json["wake_shadow_factor"].get<double>();
                    }
                    if (comp_json.contains("hysteresis_tau")) {
                        comp.hysteresis_tau = comp_json["hysteresis_tau"].get<double>();
                    }
                    if (comp_json.contains("vortex_gain")) {
                        comp.vortex_gain = comp_json["vortex_gain"].get<double>();
                    }
                    
                    config.components.push_back(comp);
                }
            }
        }
        
        // Валидация
        config.validate();
        
        return config;
    }
    catch (const json::parse_error& e) {
        throw ConfigError("JSON parsing error: " + std::string(e.what()));
    }
    catch (const json::type_error& e) {
        throw ConfigError("Type error in JSON: " + std::string(e.what()));
    }
}

AeroConfig JsonParser::parseFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw ConfigError("Failed to open configuration file: " + filepath);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    return parse(buffer.str());
}

} // namespace aero
