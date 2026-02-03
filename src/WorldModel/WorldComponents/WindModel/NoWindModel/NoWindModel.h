//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "PCH.h"
#include "../IWindModel.h"

template <typename metricType>
class NoWindModel final : public IWindModel<metricType> {
public:
    // Возвращает нулевой вектор ветра (ветер отсутствует)
    Eigen::Vector3<metricType> getWindVector(Eigen::Vector3<metricType> r, metricType t) const override;
    // Возвращает нулевую скорость ветра
    metricType getWindSpeed(Eigen::Vector3<metricType> r, metricType t) const override;
    // Возвращает нулевое направление (азимут и угол возвышения)
    std::pair<metricType, metricType> getWindDirection(Eigen::Vector3<metricType> r, metricType t) const override;
};

