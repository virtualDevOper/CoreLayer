//
// Created by 4NR_Operator_3 on 24.12.2025
//

#pragma once

/**
 * \brief Структура начальных условий объекта в симуляции
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Содержит ТОЛЬКО начальное состояние (position, velocity, orientation, angular velocity).
 * Все остальные параметры (масса, инерция, силы, моменты) вычисляются интерполяторами
 */

template<typename metricType>
struct ObjInitParams {
    ObjInitParams()
        : position(Eigen::Vector3<metricType>::Zero())
        , velocity(Eigen::Vector3<metricType>::Zero())
        , eulerAngles(Eigen::Vector3<metricType>::Zero())
        , angularVelocity(Eigen::Vector3<metricType>::Zero()) {}

    // === ПОЗИЦИЯ И ОРИЕНТАЦИЯ ===
    Eigen::Vector3<metricType> position;           // [м]
    Eigen::Vector3<metricType> velocity;           // [м/с]
    Eigen::Vector3<metricType> eulerAngles;        // [рад]
    Eigen::Vector3<metricType> angularVelocity;    // [рад/с]
};
