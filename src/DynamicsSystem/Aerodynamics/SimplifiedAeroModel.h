// src/DynamicsSystem/Aerodynamics/SimplifiedAeroModel.h
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
class SimplifiedAeroModel {
private:
    std::unique_ptr<ILinearInterpolator<metricType>> cx_before_;
    std::unique_ptr<ILinearInterpolator<metricType>> cx_after_;
    std::unique_ptr<ILinearInterpolator<metricType>> s_ref_interp_;

    metricType deploy_start_time_;
    metricType deploy_duration_;

public:
    SimplifiedAeroModel(
        std::unique_ptr<ILinearInterpolator<metricType>> cx_before,
        std::unique_ptr<ILinearInterpolator<metricType>> cx_after,
        std::unique_ptr<ILinearInterpolator<metricType>> s_ref_interp,
        metricType deploy_start = 1.5,
        metricType deploy_dur = 0.2
    );

    /**
     * @brief Вычисляет силу сопротивления в ENU-СК.
     * @param V_enu Скорость в ENU [м/с]
     * @param rho Плотность воздуха [кг/м³]
     * @param t Время [с]
     * @return Вектор силы [Н] в ENU
     */
    Eigen::Vector3<metricType> computeForceENU(
        const Eigen::Vector3<metricType>& V_enu,
        metricType rho,
        metricType t
    ) const;

    [[nodiscard]] bool isDeployed(metricType t) const {
        return t >= deploy_start_time_ + deploy_duration_;
    }
};