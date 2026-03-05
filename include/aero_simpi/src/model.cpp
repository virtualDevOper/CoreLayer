#include "model.h"
#include <cmath>
#include <algorithm>
#include <iostream>  // === FIX: Добавлено для std::cerr ===

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace aero {
//==============================================================================
// AERODYNAMICS MODEL
//==============================================================================
AerodynamicsModel::AerodynamicsModel(const AeroConfig& config) : config_(config) {
    config_.validate();

    components_.reserve(config_.components.size());
    for (const auto& comp_config : config_.components) {
        components_.push_back(ComponentFactory::create(comp_config));
    }
}

std::shared_ptr<AerodynamicsModel> AerodynamicsModel::create(const AeroConfig& config) {
    return std::make_shared<AerodynamicsModel>(config);
}

std::shared_ptr<AerodynamicsModel> AerodynamicsModel::createFromFile(const std::string& filepath) {
    auto config = JsonParser::parseFile(filepath);
    return std::make_shared<AerodynamicsModel>(config);
}

AeroOutput AerodynamicsModel::calculate(const AeroState& state) const {
    state.validate();

    AeroState state_with_history = state;
    state_with_history.alpha_prev = alpha_prev_;
    state_with_history.epsilon_prev = epsilon_prev_;

    const auto weights = calculateMachWeights(state.M);

    std::vector<AeroOutput> outputs;
    outputs.reserve(components_.size());
    for (const auto& component : components_) {
        outputs.push_back(component->calculate(state_with_history, config_.global));
    }

    applyInterference(outputs, state);

    AeroOutput result = sumComponents(outputs, state);

    alpha_prev_ = state.alpha;
    epsilon_prev_ = calculateEpsilonFromOutputs(outputs, state);

    return result;
}

AerodynamicsModel::MachWeights AerodynamicsModel::calculateMachWeights(double M) const {
    MachWeights weights;

    weights.sub = 1.0 / (1.0 + std::exp((M - 0.85) / 0.05));
    weights.sup = 1.0 / (1.0 + std::exp((0.95 - M) / 0.05));
    weights.hyper = 1.0 / (1.0 + std::exp((3.0 - M) / 0.2));

    return weights;
}

void AerodynamicsModel::applyInterference(std::vector<AeroOutput>& outputs, const AeroState& state) const {
    int wing_index = -1;
    int fin_index = -1;
    double diameter_fuselage = 0.0;
    double b_wing = 1.0;

    for (size_t i = 0; i < outputs.size(); ++i) {
        const auto& comp = components_[i];
        if (comp->getType() == ComponentType::WING) {
            wing_index = static_cast<int>(i);
            for (const auto& cfg : config_.components) {
                if (cfg.name == comp->getName()) {
                    b_wing = cfg.b_ref;
                    break;
                }
            }
        }
        else if (comp->getType() == ComponentType::FIN) {
            fin_index = static_cast<int>(i);
        }
        else if (comp->getType() == ComponentType::BODY) {
            for (const auto& cfg : config_.components) {
                if (cfg.name == comp->getName()) {
                    diameter_fuselage = cfg.diameter;
                    break;
                }
            }
        }
    }

    if (wing_index >= 0 && diameter_fuselage > 0.0) {
        const double k_wf = 1.0 + 0.15 * std::pow(diameter_fuselage / b_wing, 2);
        outputs[wing_index].Cz *= k_wf;
        outputs[wing_index].dCz_dalpha *= k_wf;
    }

    if (wing_index >= 0 && fin_index >= 0) {
        const double alpha_rad = state.alpha * M_PI / 180.0;
        const double CL_wing_norm = -outputs[wing_index].Cz / std::cos(alpha_rad);

        double wake_factor = 0.3;
        for (const auto& cfg : config_.components) {
            if (cfg.type == ComponentType::WING) {
                wake_factor = cfg.wake_shadow_factor;
                break;
            }
        }

        double q_ratio = 1.0 - wake_factor * CL_wing_norm * CL_wing_norm;
        q_ratio = std::max(q_ratio, 0.5);

        outputs[fin_index].Cz *= q_ratio;
        outputs[fin_index].dCz_dalpha *= q_ratio;
    }
}

AeroOutput AerodynamicsModel::sumComponents(const std::vector<AeroOutput>& outputs,
                                           const AeroState& state) const {
    AeroOutput result;

    for (size_t i = 0; i < outputs.size(); ++i) {
        const auto& out = outputs[i];

        result.Cx += out.Cx;
        result.Cy += out.Cy;
        result.Cz += out.Cz;
        result.mx += out.mx;
        result.my += out.my;
        result.mz += out.mz;
        result.dCx_dalpha += out.dCx_dalpha;
        result.dCz_dalpha += out.dCz_dalpha;
        result.dmy_dalpha += out.dmy_dalpha;
        result.dCy_dbeta += out.dCy_dbeta;
        result.dmz_dbeta += out.dmz_dbeta;
        result.dCz_ddelta += out.dCz_ddelta;
        result.dmy_ddelta += out.dmy_ddelta;
        result.dCz_dq += out.dCz_dq;
        result.dmy_dq += out.dmy_dq;
        result.dCy_dr += out.dCy_dr;
        result.dmz_dr += out.dmz_dr;
        result.dmx_dp += out.dmx_dp;
    }

    result.is_transonic = (state.M >= 0.8 && state.M <= 1.2);

    // === FIX #6: ФИЗИЧЕСКИ КОРРЕКТНЫЙ РАСЧЁТ X_CP ИЗ БАЛАНСА МОМЕНТОВ ===
    const double Cz_total = result.Cz;
    const double my_total = result.my;
    const double c_ref = config_.global.c_ref;

    double L_body = 1.0;
    for (const auto& cfg : config_.components) {
        if (cfg.type == ComponentType::BODY) {
            L_body = cfg.length;
            break;
        }
    }

    if (std::abs(Cz_total) > 1e-4) {
        // my = -Cz * (x_cp - x_com) / c_ref
        // Отсюда: x_cp = x_com - my * c_ref / Cz
        result.x_cp = state.x_com - my_total * c_ref / Cz_total;
    } else {
        const double nominal_static_margin = 0.20;
        result.x_cp = state.x_com + nominal_static_margin * c_ref;

        std::cerr << "[AERO DEBUG] Cz ≈ 0, using fallback x_cp = " << result.x_cp << " m" << std::endl;
    }

    // === FIX #7: EMERGENCY CLAMP ТОЛЬКО ДЛЯ АБСУРДНЫХ ЗНАЧЕНИЙ ===
    if (result.x_cp < -L_body || result.x_cp > 3.0 * L_body) {
        std::cerr << "[AERO WARNING] x_cp = " << result.x_cp
                  << " m is absurd — applying emergency fallback" << std::endl;
        const double nominal_static_margin = 0.20;
        result.x_cp = state.x_com + nominal_static_margin * c_ref;
    }

    result.static_margin = (result.x_cp - state.x_com) / c_ref;

    for (const auto& out : outputs) {
        result.is_body_stalled = result.is_body_stalled || out.is_body_stalled;
        result.is_wing_stalled = result.is_wing_stalled || out.is_wing_stalled;
    }

    return result;
}

double AerodynamicsModel::calculateEpsilonFromOutputs(const std::vector<AeroOutput>& outputs,
                                                     const AeroState& state) const {
    for (const auto& out : outputs) {
        if (out.component_type == ComponentType::WING) {
            const double alpha_rad = state.alpha * M_PI / 180.0;
            const double cos_alpha = std::cos(alpha_rad);

            if (std::abs(cos_alpha) < 0.01) {
                return 0.0;
            }

            const double CL_wing = -out.Cz / cos_alpha;
            return 0.3 * CL_wing;
        }
    }

    return 0.0;
}

} // namespace aero