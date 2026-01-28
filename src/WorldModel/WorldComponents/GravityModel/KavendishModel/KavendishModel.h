//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IGravitationalModel.h"

template <typename metricType>
class KavendishModel final : public IGravitationalModel<metricType> {
public:
    metricType getGravitationalAcceleration(Eigen::Vector3<metricType> r) const override;
};

#include "KavendishModel.tpp"

