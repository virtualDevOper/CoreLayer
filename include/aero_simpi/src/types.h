#pragma once

#include <stdexcept>
#include <string>
#include <memory>
#include <functional>
#include <Eigen/Dense>

namespace aero {

//==============================================================================
// ИСКЛЮЧЕНИЯ
//==============================================================================

/**
 * @brief Базовое исключение для всех ошибок аэродинамической модели
 */
class AeroException : public std::runtime_error {
public:
    explicit AeroException(const std::string& message)
        : std::runtime_error(message) {}
    virtual ~AeroException() = default;
};

/**
 * @brief Исключение конфигурации (ошибки JSON, отсутствующие параметры)
 */
class ConfigError : public AeroException {
public:
    explicit ConfigError(const std::string& message)
        : AeroException("[ConfigError] " + message) {}
};

/**
 * @brief Исключение сингулярности (M=1, α=90°, V=0 и т.д.)
 */
class SingularError : public AeroException {
public:
    explicit SingularError(const std::string& message)
        : AeroException("[SingularError] " + message) {}
};

/**
 * @brief Исключение диапазона (параметры вне допустимых пределов)
 */
class RangeError : public AeroException {
public:
    explicit RangeError(const std::string& message)
        : AeroException("[RangeError] " + message) {}
};

//==============================================================================
// ТИПЫ ДАННЫХ
//==============================================================================

/**
 * @brief Вектор 3D (используем Eigen для совместимости)
 */
using Vector3d = Eigen::Vector3d;

/**
 * @brief Типы аэродинамических компонентов
 */
enum class ComponentType {
    BODY,   // Тело вращения
    WING,   // Крыло
    FIN     // Руль/стабилизатор
};

/**
 * @brief Типы формы носа
 */
enum class NoseType {
    SPHERE,  // Сферический
    OGIVE,   // Оживальный
    CONE     // Конический
};

/**
 * @brief Состояние аэродинамического расчёта
 */
struct AeroState {
    // Кинематика
    double V{0.0};           // Скорость, м/с
    double alpha{0.0};       // Угол атаки, градусы
    double beta{0.0};        // Угол скольжения, градусы
    double p{0.0};           // Угловая скорость крена, рад/с
    double q{0.0};           // Угловая скорость тангажа, рад/с
    double r{0.0};           // Угловая скорость рыскания, рад/с
    
    // Атмосфера
    double rho{1.225};       // Плотность воздуха, кг/м³ (стандарт на уровне моря)
    double M{0.0};           // Число Маха
    double Re{1e6};          // Число Рейнольдса
    
    // Время
    double dt{0.01};         // Шаг интегрирования, с
    
    // Геометрия (ЦМ)
    double x_com{0.0};       // Координата ЦМ по X от носа, м
    double y_com{0.0};       // Координата ЦМ по Y, м
    double z_com{0.0};       // Координата ЦМ по Z, м
    
    // Предыдущее состояние (для гистерезиса)
    double alpha_prev{0.0};  // Предыдущий угол атаки, градусы
    double epsilon_prev{0.0}; // Предыдущий скос потока, радианы
    
    /**
     * @brief Проверка валидности состояния
     * @throws SingularError если V = 0 или rho = 0
     * @throws RangeError если параметры вне допустимых пределов
     */
    void validate() const {
        if (V <= 0.0) {
            throw SingularError("The speed V must be > 0");
        }
        if (rho <= 0.0) {
            throw SingularError("The density rho must be > 0");
        }
        if (M < 0.0 || M > 5.0) {
            throw RangeError("The Mach number must be in the range 0.0 ... 5.0 (current: " + std::to_string(M) + ")");
        }
        // === ОТКЛЮЧЕНО: Разрешаем |α| > 90° для graceful degradation ===
        // if (std::abs(alpha) > 90.0) {
        //     throw RangeError("The angle of attack should be in the range of -90° ... +90°");
        // }
        // if (std::abs(beta) > 90.0) {
        //     throw RangeError("The sliding angle should be in the range of -90° ... +90°");
        // }
    }
};

/**
 * @brief Результаты аэродинамического расчёта
 */
struct AeroOutput {
    // Идентификация компонента
    std::string component_name{};      // Имя компонента
    ComponentType component_type{ComponentType::BODY};  // Тип компонента
    
    // Основные коэффициенты (безразмерные)
    double Cx{0.0};   // Лобовое сопротивление
    double Cy{0.0};   // Боковая сила
    double Cz{0.0};   // Нормальная сила (Z-UP: подъёмная = отрицательный)
    
    // Моменты (безразмерные)
    double mx{0.0};   // Крен
    double my{0.0};   // Тангаж (my > 0 = нос вверх)
    double mz{0.0};   // Рыскание
    
    // Статические производные (1/рад)
    double dCx_dalpha{0.0};
    double dCz_dalpha{0.0};
    double dmy_dalpha{0.0};
    double dCy_dbeta{0.0};
    double dmz_dbeta{0.0};
    double dCz_ddelta{0.0};
    double dmy_ddelta{0.0};
    
    // Динамические производные (1/(рад/с))
    double dCz_dq{0.0};
    double dmy_dq{0.0};
    double dCy_dr{0.0};
    double dmz_dr{0.0};
    double dmx_dp{0.0};
    
    // Диагностические флаги
    bool is_body_stalled{false};
    bool is_wing_stalled{false};
    bool is_transonic{false};
    
    // Геометрия
    double x_cp{0.0};            // Центр давления, м (от носа)
    double static_margin{0.0};   // (x_cp - x_com) / c_ref
    
    /**
     * @brief Вычисление сил и моментов в абсолютных величинах
     * @param S_ref Опорная площадь, м²
     * @param c_ref Опорная длина, м
     * @param b_ref Размах, м
     * @return Pair<Сила, Момент> в Н и Н·м
     */
    [[nodiscard]] std::pair<Vector3d, Vector3d> getForcesAndMoments(
        double S_ref, double c_ref, double b_ref, double V, double rho) const {
        double q = 0.5 * rho * V * V;  // Скоростной напор, Па
        
        // Силы (Н)



        Vector3d forces(
            Cx * q * S_ref,
            Cy * q * S_ref,
            Cz * q * S_ref
        );
        
        // Моменты (Н·м)
        Vector3d moments(
            mx * q * S_ref * b_ref,
            my * q * S_ref * c_ref,
            mz * q * S_ref * c_ref
        );
        
        return {forces, moments};
    }
};

/**
 * @brief Глобальная конфигурация модели
 */
struct GlobalConfig {
    double S_ref{1.0};       // Опорная площадь, м²
    double c_ref{1.0};       // Опорная длина (САХ), м
    double b_ref{1.0};       // Размах крыла, м
    
    /**
     * @brief Проверка валидности конфигурации
     */
    void validate() const {
        if (S_ref <= 0.0) {
            throw ConfigError("Опорная площадь S_ref должна быть > 0");
        }
        if (c_ref <= 0.0) {
            throw ConfigError("Опорная длина c_ref должна быть > 0");
        }
        if (b_ref <= 0.0) {
            throw ConfigError("Размах b_ref должна быть > 0");
        }
    }
};

/**
 * @brief Парсер строки в NoseType
 */
inline NoseType parseNoseType(const std::string& str) {
    if (str == "SPHERE" || str == "sphere") return NoseType::SPHERE;
    if (str == "OGIVE" || str == "ogive") return NoseType::OGIVE;
    if (str == "CONE" || str == "cone") return NoseType::CONE;
    throw ConfigError("Неизвестный тип носа: " + str);
}

/**
 * @brief Парсер строки в ComponentType
 */
inline ComponentType parseComponentType(const std::string& str) {
    if (str == "BODY" || str == "body") return ComponentType::BODY;
    if (str == "WING" || str == "wing") return ComponentType::WING;
    if (str == "FIN" || str == "fin") return ComponentType::FIN;
    throw ConfigError("Неизвестный тип компонента: " + str);
}

} // namespace aero
