//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once
#include "PCH.h"

template <typename metricType>
class IGravitationalModel {
public:
    // Возвращает вектор гравитационного ускорения [m/s²] в точке r
    // Направлен к центру Земли; модуль: g = GM / (r+h)^2
    virtual metricType getGravitationalAcceleration(const Eigen::Vector3<metricType> r) const = 0;
    virtual ~IGravitationalModel() = default;
};


