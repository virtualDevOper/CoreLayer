//
// Created by 4NR_Operator_3 on 27.01.2026.
//

#pragma once
#include "../../../../PCH.h"

/**
* \brief Абстрактный интерфейс аэродинамической модели
* \tparam metricType Тип данных для метрических величин
*
* \details Позволяет подключать разные реализации аэродинамики
* (упрощённую, полную, табличную) без изменения ядра симуляции
*/


template<typename metricType>
class IAeroModel {
public:
    virtual ~IAeroModel() = default;

    // Силы в связной СК
    virtual Eigen::Vector3<metricType> computeAerodynamicForces(
        const Eigen::Vector3<metricType>& velocity_body,
        metricType air_density,
        metricType mach_number) const = 0;

    // Моменты в связной СК
    virtual Eigen::Vector3<metricType> computeAerodynamicMoments(
        const Eigen::Vector3<metricType>& velocity_body,
        const Eigen::Vector3<metricType>& angular_velocity,
        metricType air_density,
        metricType mach_number,
        const Eigen::Vector3<metricType>& euler_angles) const = 0;
};