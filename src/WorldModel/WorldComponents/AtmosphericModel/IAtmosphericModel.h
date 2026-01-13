//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../../../PCH.h"

template <typename metricType>
class IAtmosphericModel {
public:
    // Возвращает плотность воздуха [kg/m³] в точке r
    virtual metricType getDensity(const Eigen::Vector3<metricType> r) const = 0;

    // Возвращает давление [Pa] в точке r
    virtual metricType getPressure(const Eigen::Vector3<metricType> r) const = 0;

    // Возвращает температуру [K] в точке r
    virtual metricType getTemperature(const Eigen::Vector3<metricType> r) const = 0;

    // Возвращает скорость звука [m/s] в точке r
    virtual metricType getSpeedOfSound(const Eigen::Vector3<metricType> r) const = 0;

    virtual ~IAtmosphericModel() = default;
};
