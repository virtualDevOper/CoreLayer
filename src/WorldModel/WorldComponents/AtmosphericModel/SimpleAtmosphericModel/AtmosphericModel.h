//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IAtmosphericModel.h"

template <typename metricType>
class AtmosphericModel final : public IAtmosphericModel<metricType> {
public:
    AtmosphericModel() = default;
    metricType getDensity(Eigen::Vector3<metricType> r) const override;
    metricType getPressure(Eigen::Vector3<metricType> r) const override;
    metricType getTemperature(Eigen::Vector3<metricType> r) const override;
    metricType getSpeedOfSound(Eigen::Vector3<metricType> r) const override;
};
#include "AtmosphericModel.tpp"