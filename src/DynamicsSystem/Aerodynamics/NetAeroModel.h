// src/DynamicsSystem/Aerodynamics/NetAeroModel.h
#pragma once
#include "PCH.h"
#include "../../utils/Interpolation/ILinearInterpolator/ILinearInterpolator.h"

/**
 * @brief Упрощённая аэродинамика для 3DOF-модели сетевого снаряда.
 *
 * @details Только лобовое сопротивление Cx(α).
 * После раскрытия сетка всегда ориентирована "лицом" к потоку.
 */
template<typename metricType>
class NetAeroModel {
private:
    std::unique_ptr<ILinearInterpolator<metricType>> cx_before_;
    std::unique_ptr<ILinearInterpolator<metricType>> cx_after_;
    std::unique_ptr<ILinearInterpolator<metricType>> s_ref_interp_;

    metricType deploy_start_time_;
    metricType deploy_duration_;

    // Вспомогательный метод: коэффициент раскрытия [0..1]
    [[nodiscard]] metricType getDeployFactor(metricType t) const {
        if (t < deploy_start_time_) return metricType(0);
        if (t >= deploy_start_time_ + deploy_duration_) return metricType(1);
        return (t - deploy_start_time_) / deploy_duration_;
    }

public:
    NetAeroModel(
        std::unique_ptr<ILinearInterpolator<metricType>> cx_before,
        std::unique_ptr<ILinearInterpolator<metricType>> cx_after,
        std::unique_ptr<ILinearInterpolator<metricType>> s_ref_interp,
        metricType deploy_start = metricType(1.5),
        metricType deploy_dur = metricType(0.2)
    )
        : cx_before_(std::move(cx_before))
        , cx_after_(std::move(cx_after))
        , s_ref_interp_(std::move(s_ref_interp))
        , deploy_start_time_(deploy_start)
        , deploy_duration_(deploy_dur)
    {}

    /**
     * @brief Вычисляет силу сопротивления в ENU-СК.
     */
    Eigen::Vector3<metricType> computeForceENU(
        const Eigen::Vector3<metricType>& V_enu,
        metricType rho,
        metricType t
    ) const;

    /**
     * @brief Получить Cx для заданного угла атаки и времени (интерполяция до/после раскрытия).
     */
    [[nodiscard]] metricType getCx(metricType alpha_deg, metricType t) const {
        if (!cx_before_ || !cx_after_) return metricType(0);

        metricType cx_b = cx_before_->interpolate(alpha_deg);
        metricType cx_a = cx_after_->interpolate(alpha_deg);
        metricType factor = getDeployFactor(t);

        return cx_b + factor * (cx_a - cx_b);
    }

    /**
     * @brief Получить характерную площадь S_ref в момент времени t.
     */
    [[nodiscard]] metricType getSRef(metricType t) const {
        if (!s_ref_interp_) return metricType(0);
        return s_ref_interp_->interpolate(t);
    }

    [[nodiscard]] bool isDeployed(metricType t) const {
        return t >= deploy_start_time_ + deploy_duration_;
    }
};