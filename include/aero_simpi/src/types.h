#pragma once
#include <stdexcept>
#include <string>
#include <memory>
#include <functional>
#include <Eigen/Dense>
#include <iostream>  // === FIX: Добавлено для std::cerr ===

namespace aero {
//==============================================================================
// ИСКЛЮЧЕНИЯ
//==============================================================================
class AeroException : public std::runtime_error {
public:
    explicit AeroException(const std::string& message)
        : std::runtime_error(message) {}
    virtual ~AeroException() = default;
};

class ConfigError : public AeroException {
public:
    explicit ConfigError(const std::string& message)
        : AeroException("[ConfigError] " + message) {}
};

class SingularError : public AeroException {
public:
    explicit SingularError(const std::string& message)
        : AeroException("[SingularError] " + message) {}
};

class RangeError : public AeroException {
public:
    explicit RangeError(const std::string& message)
        : AeroException("[RangeError] " + message) {}
};

//==============================================================================
// ТИПЫ ДАННЫХ
//==============================================================================
using Vector3d = Eigen::Vector3d;

enum class ComponentType {
    BODY,
    WING,
    FIN
};

enum class NoseType {
    SPHERE,
    OGIVE,
    CONE
};

struct AeroState {
    double V{0.0};
    double alpha{0.0};
    double beta{0.0};
    double p{0.0};
    double q{0.0};
    double r{0.0};

    double rho{1.225};
    double M{0.0};
    double Re{1e6};

    double dt{0.01};

    double x_com{0.0};
    double y_com{0.0};
    double z_com{0.0};

    double alpha_prev{0.0};
    double epsilon_prev{0.0};

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

        // === FIX: ОТКЛЮЧЕНО ИСКЛЮЧЕНИЕ ПРИ |α| > 90° ===
        if (std::abs(alpha) > 90.0) {
            std::cerr << "[AERO WARNING] |alpha| = " << std::abs(alpha)
                      << "° exceeds 90° — using graceful degradation" << std::endl;
        }
        if (std::abs(beta) > 90.0) {
            std::cerr << "[AERO WARNING] |beta| = " << std::abs(beta)
                      << "° exceeds 90° — using graceful degradation" << std::endl;
        }

        if (std::abs(alpha) > 80.0) {
            std::cerr << "[AERO DEBUG] High angle of attack: alpha = " << alpha
                      << "°, model accuracy degraded" << std::endl;
        }
    }
};

struct AeroOutput {
    std::string component_name{};
    ComponentType component_type{ComponentType::BODY};

    double Cx{0.0};
    double Cy{0.0};
    double Cz{0.0};

    double mx{0.0};
    double my{0.0};
    double mz{0.0};

    double dCx_dalpha{0.0};
    double dCz_dalpha{0.0};
    double dmy_dalpha{0.0};
    double dCy_dbeta{0.0};
    double dmz_dbeta{0.0};
    double dCz_ddelta{0.0};
    double dmy_ddelta{0.0};

    double dCz_dq{0.0};
    double dmy_dq{0.0};
    double dCy_dr{0.0};
    double dmz_dr{0.0};
    double dmx_dp{0.0};

    bool is_body_stalled{false};
    bool is_wing_stalled{false};
    bool is_transonic{false};

    double x_cp{0.0};
    double static_margin{0.0};

    [[nodiscard]] std::pair<Vector3d, Vector3d> getForcesAndMoments(
        double S_ref, double c_ref, double b_ref, double V, double rho) const {
        double q = 0.5 * rho * V * V;

        Vector3d forces(
            Cx * q * S_ref,
            Cy * q * S_ref,
            Cz * q * S_ref
        );

        Vector3d moments(
            mx * q * S_ref * b_ref,
            my * q * S_ref * c_ref,
            mz * q * S_ref * c_ref
        );

        return {forces, moments};
    }
};

struct GlobalConfig {
    double S_ref{1.0};
    double c_ref{1.0};
    double b_ref{1.0};

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

inline NoseType parseNoseType(const std::string& str) {
    if (str == "SPHERE" || str == "sphere") return NoseType::SPHERE;
    if (str == "OGIVE" || str == "ogive") return NoseType::OGIVE;
    if (str == "CONE" || str == "cone") return NoseType::CONE;
    throw ConfigError("Неизвестный тип носа: " + str);
}

inline ComponentType parseComponentType(const std::string& str) {
    if (str == "BODY" || str == "body") return ComponentType::BODY;
    if (str == "WING" || str == "wing") return ComponentType::WING;
    if (str == "FIN" || str == "fin") return ComponentType::FIN;
    throw ConfigError("Неизвестный тип компонента: " + str);
}

} // namespace aero