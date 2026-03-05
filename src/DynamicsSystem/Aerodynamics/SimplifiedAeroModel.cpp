// src/DynamicsSystem/Aerodynamics/SimplifiedAeroModel.cpp
#include "PCH.h"
#include "SimplifiedAeroModel.h"

template<typename metricType>
SimplifiedAeroModel<metricType>::SimplifiedAeroModel(
    std::unique_ptr<ILinearInterpolator<metricType>> cx_before,
    std::unique_ptr<ILinearInterpolator<metricType>> cx_after,
    std::unique_ptr<ILinearInterpolator<metricType>> s_ref_interp,
    metricType deploy_start,
    metricType deploy_dur
) : cx_before_(std::move(cx_before)),
    cx_after_(std::move(cx_after)),
    s_ref_interp_(std::move(s_ref_interp)),
    deploy_start_time_(deploy_start),
    deploy_duration_(deploy_dur)
{
    if (!cx_before_ || !cx_after_ || !s_ref_interp_) {
        throw std::invalid_argument("All interpolators are required");
    }
}

template<typename metricType>
Eigen::Vector3<metricType> SimplifiedAeroModel<metricType>::computeForceENU(
    const Eigen::Vector3<metricType>& V_enu,
    metricType rho,
    metricType t) const
{
    const metricType V_mag = V_enu.norm();
    if (V_mag < metricType(1e-6)) {
        return Eigen::Vector3<metricType>::Zero();
    }

    // === ФАЗА РАСКРЫТИЯ [0..1] ===
    metricType deploy_factor = metricType(0);
    if (t >= deploy_start_time_) {
        deploy_factor = std::min(metricType(1),
            (t - deploy_start_time_) / deploy_duration_);
    }

    // === УГОЛ АТАКИ (для компактного снаряда) ===
    // Для сетки после раскрытия alpha не важен — она всегда "лицом" к потоку
    metricType alpha_deg = metricType(0);
    if (deploy_factor < metricType(0.99)) {
        // До раскрытия: alpha = угол между V и осью X (предполагаем, что снаряд летит носом вперёд)
        alpha_deg = std::atan2(-V_enu.z(), V_enu.x()) * MathConstants::RAD_TO_DEG;
    }

    // === ИНТЕРПОЛЯЦИЯ Cx ===
    const metricType Cx_b = cx_before_->interpolate(alpha_deg);
    const metricType Cx_a = cx_after_->interpolate(metricType(0)); // сетка: alpha=0
    const metricType Cx = Cx_b + deploy_factor * (Cx_a - Cx_b);

    // === ПЛОЩАСТЬ ИЗ ТАБЛИЦЫ ===
    const metricType S_ref = s_ref_interp_->interpolate(t);

    // === ДИНАМИЧЕСКОЕ ДАВЛЕНИЕ ===
    const metricType q_dyn = metricType(0.5) * rho * V_mag * V_mag;

    // === СИЛА: направлена ПРОТИВ скорости в ENU ===
    // F = -Cx * q * S * (V_normalized)
    return -Cx * q_dyn * S_ref * (V_enu / V_mag);
}

template class SimplifiedAeroModel<GLOBAL_CONFIG::PROJECT_TYPE>;