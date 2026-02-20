#include "model.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace aero {

//==============================================================================
// AERODYNAMICS MODEL
//==============================================================================

AerodynamicsModel::AerodynamicsModel(const AeroConfig& config) : config_(config) {
    // Валидация конфигурации
    config_.validate();
    
    // Создание компонентов через фабрику
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
    // Валидация состояния
    state.validate();
    
    // Создаём локальную копию state для передачи в компоненты с историей гистерезиса
    AeroState state_with_history = state;
    state_with_history.alpha_prev = alpha_prev_;      // ← из кэша модели
    state_with_history.epsilon_prev = epsilon_prev_;  // ← из кэша модели
    
    // Расчёт весов по Маху
    const auto weights = calculateMachWeights(state.M);
    
    // Расчёт каждого компонента (они читают state_with_history.alpha_prev)
    std::vector<AeroOutput> outputs;
    outputs.reserve(components_.size());
    for (const auto& component : components_) {
        outputs.push_back(component->calculate(state_with_history, config_.global));
    }
    
    // Применение интерференции
    applyInterference(outputs, state);
    
    // Гибридное суммирование
    AeroOutput result = sumComponents(outputs, state);
    
    // Обновление кэша для следующего шага
    // ВАЖНО: Обновляем ПОСЛЕ расчёта, чтобы текущий шаг использовал предыдущие значения
    alpha_prev_ = state.alpha;
    epsilon_prev_ = calculateEpsilonFromOutputs(outputs, state);
    
    return result;
}

AerodynamicsModel::MachWeights AerodynamicsModel::calculateMachWeights(double M) const {
    MachWeights weights;
    
    // Сигмоидальные веса для плавной интерполяции
    // Дозвук: M < 0.85
    weights.sub = 1.0 / (1.0 + std::exp((M - 0.85) / 0.05));
    
    // Сверхзвук: M > 0.95
    weights.sup = 1.0 / (1.0 + std::exp((0.95 - M) / 0.05));
    
    // Гиперзвук: M > 3.0
    weights.hyper = 1.0 / (1.0 + std::exp((3.0 - M) / 0.2));
    
    // Нормализация (опционально, для гарантии sum = 1 в переходных зонах)
    // В данной реализации веса могут суммироваться > 1 в переходных зонах,
    // что допустимо для плавной интерполяции
    
    return weights;
}

void AerodynamicsModel::applyInterference(std::vector<AeroOutput>& outputs, const AeroState& state) const {
    // Поиск индексов компонентов по типу
    int wing_index = -1;
    int fin_index = -1;
    double diameter_fuselage = 0.0;
    double b_wing = 1.0;
    
    for (size_t i = 0; i < outputs.size(); ++i) {
        const auto& comp = components_[i];
        if (comp->getType() == ComponentType::WING) {
            wing_index = static_cast<int>(i);
            // Находим размах крыла из конфигурации
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
            // Находим диаметр фюзеляжа из конфигурации
            for (const auto& cfg : config_.components) {
                if (cfg.name == comp->getName()) {
                    diameter_fuselage = cfg.diameter;
                    break;
                }
            }
        }
    }
    
    // Интерференция крыло-фюзеляж
    if (wing_index >= 0 && diameter_fuselage > 0.0) {
        const double k_wf = 1.0 + 0.15 * std::pow(diameter_fuselage / b_wing, 2);
        outputs[wing_index].Cz *= k_wf;
        outputs[wing_index].dCz_dalpha *= k_wf;
    }
    
    // След за крылом (экранирование ГО)
    if (wing_index >= 0 && fin_index >= 0) {
        const double alpha_rad = state.alpha * M_PI / 180.0;
        const double CL_wing_norm = -outputs[wing_index].Cz / std::cos(alpha_rad);
        
        // Находим wake_shadow_factor из конфигурации
        double wake_factor = 0.3;
        for (const auto& cfg : config_.components) {
            if (cfg.type == ComponentType::WING) {
                wake_factor = cfg.wake_shadow_factor;
                break;
            }
        }
        
        double q_ratio = 1.0 - wake_factor * CL_wing_norm * CL_wing_norm;
        q_ratio = std::max(q_ratio, 0.5);  // Минимум 50% напора
        
        outputs[fin_index].Cz *= q_ratio;
        outputs[fin_index].dCz_dalpha *= q_ratio;
    }
}

AeroOutput AerodynamicsModel::sumComponents(const std::vector<AeroOutput>& outputs,
                                           const AeroState& state) const {
    AeroOutput result;
    
    // Веса по типу компонента
    double w_body = 1.0;
    double w_wing = 1.0;
    double w_fin = 1.0;
    
    // Вес крыла падает при больших α и малых M для ракет
    if (std::abs(state.alpha) > 20.0) {
        w_wing *= 0.5;
    }
    
    // Вес рулей падает при срыве
    for (const auto& out : outputs) {
        if (out.is_wing_stalled) {
            w_fin *= out.is_wing_stalled ? 0.7 : 1.0;
        }
    }
    
    // Веса по Маху
    const auto mach_weights = calculateMachWeights(state.M);
    
    // Суммирование коэффициентов
    // Теперь каждый AeroOutput содержит component_name и component_type для надёжной связи
    for (size_t i = 0; i < outputs.size(); ++i) {
        const auto& out = outputs[i];
        // Выбор веса в зависимости от типа компонента
        double type_weight = 1.0;
        ComponentType comp_type = out.component_type;  // ← Используем из AeroOutput
        
        // Упрощённое суммирование (веса применяются внутри компонентов)
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
    
    // Диагностика
    result.is_transonic = (state.M >= 0.8 && state.M <= 1.2);
    
    // Вычисление общего центра давления (взвешенное среднее)
    double total_Cz = std::abs(result.Cz);
    if (total_Cz > 1e-6) {
        double x_cp_sum = 0.0;
        for (const auto& out : outputs) {
            const double weight = std::abs(out.Cz) / total_Cz;
            x_cp_sum += out.x_cp * weight;
        }
        result.x_cp = x_cp_sum;
    } else {
        // При нулевой подъёмной силе берём среднее арифметическое
        double x_cp_sum = 0.0;
        for (const auto& out : outputs) {
            x_cp_sum += out.x_cp;
        }
        result.x_cp = x_cp_sum / static_cast<double>(outputs.size());
    }
    
    // Статический запас устойчивости
    result.static_margin = (result.x_cp - state.x_com) / config_.global.c_ref;
    
    // Флаги сваливания
    for (const auto& out : outputs) {
        result.is_body_stalled = result.is_body_stalled || out.is_body_stalled;
        result.is_wing_stalled = result.is_wing_stalled || out.is_wing_stalled;
    }
    
    return result;
}

double AerodynamicsModel::calculateEpsilonFromOutputs(const std::vector<AeroOutput>& outputs,
                                                     const AeroState& state) const {
    // Скос потока (downwash) вычисляется на основе подъёмной силы крыла
    // Формула: epsilon ≈ k * CL_wing, где k ≈ 0.3 для типичных конфигураций
    
    // Найти wing output
    for (const auto& out : outputs) {
        if (out.component_type == ComponentType::WING) {
            // Вычисляем CL крыла из Cz
            const double alpha_rad = state.alpha * M_PI / 180.0;
            const double cos_alpha = std::cos(alpha_rad);
            
            // Защита от деления на ноль при alpha ≈ 90°
            if (std::abs(cos_alpha) < 0.01) {
                return 0.0;
            }
            
            const double CL_wing = -out.Cz / cos_alpha;
            
            // Скос потока пропорционален подъёмной силе
            // Коэффициент 0.3 - типичное значение для крыло-хвост конфигурации
            return 0.3 * CL_wing;
        }
    }
    
    // Если крыла нет, скос потока нулевой
    return 0.0;
}

} // namespace aero
