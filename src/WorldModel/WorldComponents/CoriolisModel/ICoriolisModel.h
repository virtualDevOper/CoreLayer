//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once
#include "PCH.h"

template <typename metricType>
class ICoriolisModel {
    public:
    virtual Eigen::Vector3<metricType> getCoriolisForce(Eigen::Vector3<metricType> r) const = 0;
    virtual ~ICoriolisModel()= default;
};



