//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once
#include "PCH.h"
#include "../ICoriolisModel.h"

template <typename metricType>
class NoCoriolisForceModel final : public ICoriolisModel<metricType> {
    public:
    Eigen::Vector3<metricType> getCoriolisForce(const Eigen::Vector3<metricType> r) const override;
};

