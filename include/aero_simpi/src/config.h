#pragma once

#include "types.h"
#include <string>
#include <vector>

namespace aero {

/**
 * @brief Конфигурация отдельного аэродинамического компонента
 */
struct ComponentConfig {
    std::string name;                    // Имя компонента
    ComponentType type{ComponentType::BODY}; // Тип компонента
    
    // Геометрия
    double length{1.0};                  // Длина, м (BODY)
    double diameter{0.1};                // Диаметр, м (BODY)
    double S_ref{0.1};                   // Опорная площадь, м² (WING, FIN)
    double b_ref{1.0};                   // Размах, м
    double c_ref{0.1};                   // Хорда/длина, м
    double AR{1.0};                      // Удлинение = b_ref² / S_ref
    
    // Форма
    NoseType nose_type{NoseType::OGIVE}; // Тип носа (BODY)
    
    // Позиция (от носа)
    double x_pos{0.0};                   // Позиция по X, м
    double y_pos{0.0};                   // Позиция по Y, м
    double z_pos{0.0};                   // Позиция по Z, м
    
    // Рули
    bool can_deflect{false};             // Может ли отклоняться
    double delta{0.0};                   // Угол отклонения, градусы
    double mount_angle{0.0};             // Угол установки вокруг оси X, градусы
    
    // Настраиваемые параметры
    double k_interference{1.15};         // Коэффициент интерференции
    double wake_shadow_factor{0.3};      // Фактор экранирования следом
    double hysteresis_tau{0.2};          // Постоянная времени гистерезиса, с
    double vortex_gain{0.1};             // Коэффициент вихревой подъёмной силы
    
    // Валидация
    void validate() const {
        if (length <= 0.0 && type == ComponentType::BODY) {
            throw ConfigError("Body length should be > 0: " + name);
        }
        if (diameter <= 0.0 && type == ComponentType::BODY) {
            throw ConfigError("The body diameter must be > 0: " + name);
        }
        if (S_ref <= 0.0 && (type == ComponentType::WING || type == ComponentType::FIN)) {
            throw ConfigError("The support area must be > 0: " + name);
        }
    }
};

/**
 * @brief Полная конфигурация аэродинамической модели
 */
struct AeroConfig {
    std::string name{"Unnamed"};         // Имя конфигурации
    GlobalConfig global;                  // Глобальные параметры
    std::vector<ComponentConfig> components; // Компоненты
    
    // Центр масс (дублируется из AeroState для удобства)
    double x_com{0.0};
    double y_com{0.0};
    double z_com{0.0};
    
    /**
     * @brief Проверка валидности всей конфигурации
     */
    void validate() const {
        global.validate();
        if (components.empty()) {
            throw ConfigError("The configuration must contain at least one component.");
        }
        for (const auto& comp : components) {
            comp.validate();
        }
    }
};

} // namespace aero
