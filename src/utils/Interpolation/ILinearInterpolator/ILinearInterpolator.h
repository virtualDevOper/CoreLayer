//
// Created by 4NR_Operator_3 on 19.11.2025.
//
#pragma once

template<typename metricType>
class ILinearInterpolator {
public:
    virtual ~ILinearInterpolator() = default;
    virtual metricType interpolate(metricType x) const = 0;
};

