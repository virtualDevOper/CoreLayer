//
// Created by 4NR_Operator_3 on 19.11.2025.
//

#pragma once

template<typename metricType>
class IBilinearInterpolator {
public:
    virtual ~IBilinearInterpolator() = default;
    virtual metricType interpolate(metricType x, const metricType y) const = 0;
};
