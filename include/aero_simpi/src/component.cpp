#include "component.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// === FIX #9: Определяем M_PI_2 если не определён ===
#ifndef M_PI_2
#define M_PI_2 (M_PI / 2.0)
#endif

namespace aero {
constexpr double MIN_VELOCITY = 0.1;

//==============================================================================
// BODY COMPONENT
//==============================================================================
BodyComponent::BodyComponent(const ComponentConfig& config)
    : AerodynamicComponent(config) {
    if (config.type != ComponentType::BODY) {
        throw ConfigError("BodyComponent ensures COMPONENT::BODY type");
    }
}

AeroOutput BodyComponent::calculate(const AeroState& state,
                                   const GlobalConfig& global) const {
    AeroOutput output;

    output.component_name = config_.name;
    output.component_type = ComponentType::BODY;

    const double alpha_rad = deg2rad(state.alpha);
    const double beta_rad  = deg2rad(state.beta);
    const double alpha_rad_abs = std::abs(alpha_rad);

    const double diameter = config_.diameter;
    const double length   = config_.length;
    const double S_mid    = M_PI * diameter * diameter / 4.0;
    const double S_wetted = M_PI * diameter * length + S_mid;

    const double Re = std::max(state.Re, 1e4);
    const double logRe = std::log10(Re);
    const double Cf = 0.455 / std::pow(logRe, 2.58);

    output.Cx = Cf * S_wetted / global.S_ref;

    double Cx_base = 0.0;
    switch (config_.nose_type) {
        case NoseType::SPHERE: Cx_base = 0.12; break;
        case NoseType::OGIVE:  Cx_base = 0.08; break;
        case NoseType::CONE:   Cx_base = 0.15; break;
    }
    output.Cx += Cx_base;

    double k_normal = 0.0;
    switch (config_.nose_type) {
        case NoseType::SPHERE: k_normal = 6.0; break;
        case NoseType::OGIVE:  k_normal = 5.5; break;
        case NoseType::CONE:   k_normal = 4.5; break;
    }

    const double f_nonlinear = std::max(0.0, 1.0 - 0.003 * std::abs(state.alpha));
    const double CN_body = k_normal * std::sin(alpha_rad) * f_nonlinear;

    output.Cx += CN_body * std::sin(alpha_rad);

    double Cx_wave = 0.0;
    const double M = state.M;
    if (M >= 0.8 && M <= 1.2) {
        const double t = (M - 0.8) / 0.4;
        const double sigmoid = 1.0 / (1.0 + std::exp(-10.0 * (t - 0.5)));
        const double supersonic_part = 0.1 * std::pow(M - 1.0, 2);
        Cx_wave = supersonic_part * sigmoid;
    }
    else if (M > 1.2) {
        const double t_c = diameter / length;
        Cx_wave = 4.0 * t_c * t_c / std::sqrt(std::max(M * M - 1.0, 0.01));
        if (M > 3.0) {
            const double newton_factor = 2.0 * std::sin(alpha_rad) * std::sin(alpha_rad);
            Cx_wave *= (1.0 + 0.5 * newton_factor);
        }
    }
    output.Cx += Cx_wave;

    output.Cz = -CN_body * std::cos(alpha_rad);
    output.Cy = -k_normal * std::sin(beta_rad) * std::cos(beta_rad) * f_nonlinear;

    double x_cp_base = 0.65 * length;
    double x_cp_mach = x_cp_base * (1.0 + 0.1 * std::max(0.0, M - 1.0));

    const double max_cp_shift_fraction = 0.15;
    const double tau = 0.3;
    const double shift_fraction = max_cp_shift_fraction * (1.0 - std::exp(-alpha_rad_abs / tau));

    output.x_cp = x_cp_mach * (1.0 - shift_fraction);

    const double min_x_cp = 0.10 * length;
    const double max_x_cp = 0.95 * length;
    output.x_cp = std::clamp(output.x_cp, min_x_cp, max_x_cp);

    const double arm_x = output.x_cp - state.x_com;

    output.my = -output.Cz * arm_x / global.c_ref;
    output.mz = -output.Cy * arm_x / global.c_ref;
    output.mx = 0.0;

    const double dCN_dalpha = k_normal * (std::cos(alpha_rad) * f_nonlinear
        - std::sin(alpha_rad) * 0.003 * sign(state.alpha) * M_PI/180.0);
    output.dCx_dalpha = dCN_dalpha * std::sin(alpha_rad) + CN_body * std::cos(alpha_rad);
    output.dCz_dalpha = -dCN_dalpha * std::cos(alpha_rad) + CN_body * std::sin(alpha_rad);
    output.dCy_dbeta = -k_normal * std::cos(2.0 * beta_rad);

    // === КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: полная производная x_cp по α ===
    const double exp_term = std::exp(-alpha_rad_abs / tau);
    const double dx_cp_dalpha = -x_cp_mach
        * max_cp_shift_fraction
        * exp_term
        * (1.0 / tau)
        * sign(state.alpha);

    // === КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: полная производная момента ===
    // dmy/dα = -dCz/dα * arm/c - Cz * dx_cp/dα / c
    output.dmy_dalpha = -output.dCz_dalpha * arm_x / global.c_ref
                       - output.Cz * dx_cp_dalpha / global.c_ref;  // ← ДОБАВЛЕНО ВТОРОЕ СЛАГАЕМОЕ!

    output.dmz_dbeta = -output.dCy_dbeta * arm_x / global.c_ref;

    if (alpha_rad_abs > 0.1) {
        const double expected_sign = -sign(state.alpha);
        if (output.dmy_dalpha * expected_sign < 0.0) {
            output.dmy_dalpha = expected_sign * std::max(std::abs(output.dmy_dalpha), 0.1);
        }
    }

    const double CL_alpha_body = k_normal * M_PI / 180.0;

    output.dCz_dq = -CL_alpha_body * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5;

    const double damping_from_normal = -2.0 * dCN_dalpha * (arm_x / global.c_ref) * (arm_x / global.c_ref);
    const double damping_from_friction = -0.1 * (length / global.c_ref) * (length / global.c_ref);
    const double wetted_area = M_PI * diameter * length;
    const double viscous_damping = -Cf * wetted_area / global.S_ref * (length / global.c_ref);
    output.dmy_dq = damping_from_normal + damping_from_friction + viscous_damping * 0.1;

    output.dmx_dp = -Cf * wetted_area / global.S_ref * 0.05;
    output.dCy_dr = CL_alpha_body * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5;
    output.dmz_dr = -output.dCy_dr * arm_x / global.c_ref;

    // === FIX #10: M_PI_2 → M_PI / 2.0 ===
    const double alpha_deg_abs = std::abs(state.alpha);
    const double fade_alpha = (alpha_deg_abs > 80.0)
        ? std::cos(std::min(1.0, (alpha_deg_abs - 80.0) / 10.0) * (M_PI / 2.0))
        : 1.0;

    output.Cz *= fade_alpha;
    output.Cy *= fade_alpha;
    output.Cx = output.Cx * fade_alpha + (1.0 - fade_alpha) * 2.0;
    output.my *= fade_alpha;
    output.mz *= fade_alpha;

    output.dCz_dalpha *= fade_alpha;
    output.dmy_dalpha *= fade_alpha;
    output.dmy_dq *= fade_alpha;
    output.dmx_dp *= fade_alpha;
    output.dmz_dr *= fade_alpha;

    output.is_body_stalled = (alpha_deg_abs > 30.0);
    output.is_transonic = (M >= 0.8 && M <= 1.2);
    output.static_margin = (output.x_cp - state.x_com) / global.c_ref;

    return output;
}

//==============================================================================
// WING COMPONENT
//==============================================================================
WingComponent::WingComponent(const ComponentConfig& config)
    : AerodynamicComponent(config) {
    if (config.type != ComponentType::WING) {
        throw ConfigError("WingComponent ensure COMPONENT::WING type");
    }
}

AeroOutput WingComponent::calculate(const AeroState& state,
                                   const GlobalConfig& global) const {
    AeroOutput output;

    output.component_name = config_.name;
    output.component_type = ComponentType::WING;

    const double alpha_rad = deg2rad(state.alpha);
    const double beta_rad  = deg2rad(state.beta);
    const double alpha_rad_abs = std::abs(alpha_rad);
    const double M = state.M;
    const double Re = std::max(state.Re, 1e4);
    const double AR = config_.AR;

    const double CL_alpha_2D = 2.0 * M_PI;
    double CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (M_PI * AR));
    double CL_alpha = 0.0;

    if (M < 0.8) {
        const double beta_PG = std::sqrt(std::max(1.0 - M * M, 0.01));
        CL_alpha = CL_alpha_3D / beta_PG;
    }
    else if (M <= 1.2) {
        const double t = (M - 0.8) / 0.4;
        const double beta_PG = std::sqrt(std::max(1.0 - M * M, 0.01));
        const double subsonic = CL_alpha_3D / beta_PG;
        const double supersonic = CL_alpha_3D / std::sqrt(std::max(M * M - 1.0, 0.01));
        const double sigmoid = 1.0 / (1.0 + std::exp(-10.0 * (t - 0.5)));
        CL_alpha = subsonic * (1.0 - sigmoid) + supersonic * sigmoid;
    }
    else {
        CL_alpha = CL_alpha_3D / std::sqrt(std::max(M * M - 1.0, 0.01));
    }

    if (M > 3.0) {
        CL_alpha *= std::max(0.5, 1.0 - 0.1 * (M - 3.0));
    }

    double alpha_stall_base = 12.0 + 2.0 * std::log10(Re / 1e6);
    alpha_stall_base = std::clamp(alpha_stall_base, 10.0, 16.0);

    const double alpha_eff = state.alpha_prev * std::exp(-state.dt / config_.hysteresis_tau)
        + state.alpha * (1.0 - std::exp(-state.dt / config_.hysteresis_tau));

    output.is_wing_stalled = (std::abs(alpha_eff) > alpha_stall_base);

    double stall_factor = 1.0;
    if (output.is_wing_stalled) {
        const double alpha_excess = std::abs(alpha_eff) - alpha_stall_base;
        if (alpha_excess < 5.0) {
            stall_factor = 1.0 - 0.02 * alpha_excess;
        } else if (alpha_excess < 15.0) {
            stall_factor = 0.9 - 0.05 * (alpha_excess - 5.0);
        } else {
            stall_factor = 0.4;
        }
        stall_factor = std::max(stall_factor, 0.4);
    }

    const double CL_wing = CL_alpha * alpha_rad * stall_factor * config_.k_interference;

    double CL_vortex = 0.0;
    if (std::abs(state.alpha) > 15.0) {
        CL_vortex = config_.vortex_gain * std::sin(2.0 * alpha_rad)
            * (std::abs(state.alpha) - 15.0) / 30.0;
    }

    const double CL_total = CL_wing + CL_vortex;

    double dCL_dalpha = CL_alpha * stall_factor;
    if (std::abs(state.alpha) > 15.0) {
        dCL_dalpha += config_.vortex_gain * 2.0 * std::cos(2.0 * alpha_rad)
            * (std::abs(state.alpha) - 15.0) / 30.0;
        dCL_dalpha += config_.vortex_gain * std::sin(2.0 * alpha_rad)
            * sign(state.alpha) / 30.0 * M_PI/180.0;
    }

    output.Cz = -CL_total * std::cos(alpha_rad);
    output.dCz_dalpha = -dCL_dalpha * std::cos(alpha_rad) + CL_total * std::sin(alpha_rad);

    const double Cf = 0.455 / std::pow(std::log10(Re), 2.58);
    output.Cx = Cf * 2.0;

    const double e_oswald = 0.85;
    output.Cx += CL_total * CL_total / (M_PI * AR * e_oswald);

    if (M >= 0.7 && M <= 1.3) {
        output.Cx += 0.02 * std::exp(-std::pow((M - 0.95) / 0.15, 2));
    }
    else if (M > 1.3) {
        const double t_c = 0.12;
        output.Cx += 4.0 * t_c * t_c / std::sqrt(std::max(M * M - 1.0, 0.01));
        if (M > 3.0) {
            output.Cx *= (1.0 + 0.2 * (M - 3.0));
        }
    }

    output.Cx += CL_total * std::sin(alpha_rad);

    output.Cy = -CL_alpha * beta_rad * stall_factor;
    output.dCy_dbeta = -CL_alpha * stall_factor;

    output.x_cp = config_.x_pos + 0.25 * config_.c_ref;

    const double arm_x = output.x_cp - state.x_com;

    output.my = -output.Cz * arm_x / global.c_ref;
    output.mz = -output.Cy * arm_x / global.c_ref;
    output.mx = 0.0;

    output.dmy_dalpha = -output.dCz_dalpha * arm_x / global.c_ref;
    output.dmz_dbeta = -output.dCy_dbeta * arm_x / global.c_ref;

    if (alpha_rad_abs > 0.1) {
        const double expected_sign = -sign(state.alpha);
        if (output.dmy_dalpha * expected_sign < 0.0) {
            output.dmy_dalpha = expected_sign * std::max(std::abs(output.dmy_dalpha), 0.1);
        }
    }

    output.dCz_dq = -CL_alpha * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5;
    output.dmy_dq = output.dCz_dq * arm_x / global.c_ref;
    output.dmx_dp = -CL_alpha * config_.b_ref * config_.b_ref
        / (4.0 * std::max(state.V, MIN_VELOCITY) * global.b_ref) * 0.5;
    output.dCy_dr = CL_alpha * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5;
    output.dmz_dr = -output.dCy_dr * arm_x / global.c_ref;

    // === FIX #11: M_PI_2 → M_PI / 2.0 ===
    const double alpha_deg_abs = std::abs(state.alpha);
    const double fade_alpha = (alpha_deg_abs > 80.0)
        ? std::cos(std::min(1.0, (alpha_deg_abs - 80.0) / 10.0) * (M_PI / 2.0))
        : 1.0;

    output.Cz *= fade_alpha;
    output.Cy *= fade_alpha;
    output.Cx = output.Cx * fade_alpha + (1.0 - fade_alpha) * 2.0;
    output.my *= fade_alpha;
    output.mz *= fade_alpha;
    output.dCz_dalpha *= fade_alpha;
    output.dmy_dalpha *= fade_alpha;
    output.dmy_dq *= fade_alpha;
    output.dmx_dp *= fade_alpha;
    output.dmz_dr *= fade_alpha;

    output.is_transonic = (M >= 0.8 && M <= 1.2);
    output.static_margin = (output.x_cp - state.x_com) / global.c_ref;

    return output;
}

//==============================================================================
// FIN COMPONENT
//==============================================================================
FinComponent::FinComponent(const ComponentConfig& config)
    : AerodynamicComponent(config) {
    if (config.type != ComponentType::FIN) {
        throw ConfigError("FinComponent ensure COMPONENT::FIN type");
    }
}

AeroOutput FinComponent::calculate(const AeroState& state,
                                  const GlobalConfig& global) const {
    AeroOutput output;

    output.component_name = config_.name;
    output.component_type = ComponentType::FIN;

    const double alpha_rad = deg2rad(state.alpha);
    const double delta_rad = deg2rad(config_.delta);
    const double mount_angle_rad = deg2rad(config_.mount_angle);
    const double M = state.M;
    const double Re = std::max(state.Re, 1e4);
    const double AR = config_.AR;

    const double CL_alpha_2D = 2.0 * M_PI;
    double CL_alpha_3D = CL_alpha_2D / (1.0 + CL_alpha_2D / (M_PI * AR));
    double CL_alpha = 0.0;

    if (M < 0.8) {
        const double beta_PG = std::sqrt(std::max(1.0 - M * M, 0.01));
        CL_alpha = CL_alpha_3D / beta_PG;
    }
    else if (M <= 1.2) {
        const double t = (M - 0.8) / 0.4;
        const double beta_PG = std::sqrt(std::max(1.0 - M * M, 0.01));
        const double subsonic = CL_alpha_3D / beta_PG;
        const double supersonic = CL_alpha_3D / std::sqrt(std::max(M * M - 1.0, 0.01));
        const double sigmoid = 1.0 / (1.0 + std::exp(-10.0 * (t - 0.5)));
        CL_alpha = subsonic * (1.0 - sigmoid) + supersonic * sigmoid;
    }
    else {
        CL_alpha = CL_alpha_3D / std::sqrt(std::max(M * M - 1.0, 0.01));
    }

    if (M > 3.0) {
        CL_alpha *= std::max(0.5, 1.0 - 0.1 * (M - 3.0));
    }

    double alpha_stall_base = 12.0 + 2.0 * std::log10(Re / 1e6);
    alpha_stall_base = std::clamp(alpha_stall_base, 10.0, 16.0);

    const double alpha_eff = state.alpha_prev * std::exp(-state.dt / config_.hysteresis_tau)
        + state.alpha * (1.0 - std::exp(-state.dt / config_.hysteresis_tau));

    const bool is_stalled = (std::abs(alpha_eff) > alpha_stall_base);

    double stall_factor = 1.0;
    if (is_stalled) {
        const double alpha_excess = std::abs(alpha_eff) - alpha_stall_base;
        stall_factor = std::max(0.4, 1.0 - 0.02 * std::min(alpha_excess, 30.0));
    }

    double eta_base = 0.7;
    double eta_mach = 1.0;
    if (M >= 0.7 && M < 1.3) {
        eta_mach = 1.0 - 0.3 * (M - 0.7) / 0.6;
    } else if (M >= 1.3 && M <= 3.0) {
        eta_mach = 0.6;
    } else if (M > 3.0) {
        eta_mach = std::max(0.4, 0.6 - 0.05 * (M - 3.0));
    }

    const double eta_alpha = 1.0 - 0.5 * std::sin(alpha_rad) * std::sin(alpha_rad);
    const double eta = eta_base * eta_mach * eta_alpha;

    const double epsilon_eff = state.epsilon_prev * std::exp(-state.dt / 0.1)
        + 0.3 * (-state.alpha * M_PI / 180.0) * (1.0 - std::exp(-state.dt / 0.1));

    const double alpha_fin_eff = alpha_rad - epsilon_eff;

    const double CL_fin = CL_alpha * (alpha_fin_eff + delta_rad) * stall_factor * eta;

    output.Cz = -CL_fin * std::cos(mount_angle_rad) * config_.S_ref / global.S_ref;
    output.Cy = -CL_fin * std::sin(mount_angle_rad) * config_.S_ref / global.S_ref;

    const double dCL_ddelta = CL_alpha * eta * stall_factor;
    output.dCz_ddelta = -dCL_ddelta * std::cos(mount_angle_rad) * config_.S_ref / global.S_ref;
    output.dmy_ddelta = -output.dCz_ddelta * (config_.x_pos - state.x_com) / global.c_ref;

    const double Cf = 0.455 / std::pow(std::log10(Re), 2.58);
    output.Cx = Cf * 2.0 + CL_fin * CL_fin / (M_PI * AR * 0.85);

    const double arm_x = config_.x_pos - state.x_com;
    output.my = -output.Cz * arm_x / global.c_ref;
    output.mz = -output.Cy * arm_x / global.c_ref;
    output.mx = (output.Cz * config_.y_pos - output.Cy * config_.z_pos) * arm_x / global.b_ref;

    output.dCz_dalpha = -CL_alpha * stall_factor * eta * std::cos(mount_angle_rad)
        * config_.S_ref / global.S_ref;
    output.dmy_dalpha = -output.dCz_dalpha * arm_x / global.c_ref;

    const double alpha_rad_abs = std::abs(alpha_rad);
    if (alpha_rad_abs > 0.1) {
        const double expected_sign = -sign(state.alpha);
        if (output.dmy_dalpha * expected_sign < 0.0) {
            output.dmy_dalpha = expected_sign * std::max(std::abs(output.dmy_dalpha), 0.1);
        }
    }

    const double damping_base = -2.0 * CL_alpha * stall_factor * eta
        * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5
        * (arm_x / global.c_ref) * config_.S_ref / global.S_ref;

    output.dmy_dq = damping_base * std::cos(mount_angle_rad);
    output.dmz_dr = damping_base * std::sin(mount_angle_rad);

    const double roll_arm = std::sqrt(config_.y_pos * config_.y_pos + config_.z_pos * config_.z_pos);
    output.dmx_dp = -CL_alpha * stall_factor * eta
        * (roll_arm / std::max(state.V, MIN_VELOCITY)) * 0.5
        * (roll_arm / global.b_ref) * config_.S_ref / global.S_ref;
    output.dCy_dr = CL_alpha * (arm_x / std::max(state.V, MIN_VELOCITY)) * 0.5
        * std::sin(mount_angle_rad) * config_.S_ref / global.S_ref;

    // === FIX #12: M_PI_2 → M_PI / 2.0 ===
    const double alpha_deg_abs = std::abs(state.alpha);
    const double fade_alpha = (alpha_deg_abs > 80.0)
        ? std::cos(std::min(1.0, (alpha_deg_abs - 80.0) / 10.0) * (M_PI / 2.0))
        : 1.0;

    output.Cz *= fade_alpha;
    output.Cy *= fade_alpha;
    output.Cx = output.Cx * fade_alpha + (1.0 - fade_alpha) * 2.0;
    output.my *= fade_alpha;
    output.mz *= fade_alpha;
    output.dmy_dalpha *= fade_alpha;
    output.dmy_dq *= fade_alpha;
    output.dmz_dr *= fade_alpha;
    output.dmx_dp *= fade_alpha;

    output.is_transonic = (M >= 0.8 && M <= 1.2);
    output.x_cp = config_.x_pos + 0.25 * config_.c_ref;
    output.static_margin = (output.x_cp - state.x_com) / global.c_ref;

    return output;
}

//==============================================================================
// COMPONENT FACTORY
//==============================================================================
std::shared_ptr<IAerodynamicComponent> ComponentFactory::create(const ComponentConfig& config) {
    switch (config.type) {
        case ComponentType::BODY:
            return std::make_shared<BodyComponent>(config);
        case ComponentType::WING:
            return std::make_shared<WingComponent>(config);
        case ComponentType::FIN:
            return std::make_shared<FinComponent>(config);
        default:
            throw ConfigError("Unknown component type: " + config.name);
    }
}

} // namespace aero