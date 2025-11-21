//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../../../PCH.h"

template <typename metricType>
class IWindModel {
public:
    virtual ~IWindModel() = default;

    // Абстрактный метод: возвращает вектор ветра в точке r в момент t
    virtual Eigen::Vector3<metricType> getWindVector( Eigen::Vector3<metricType> r, metricType t) const = 0;

    // Опционально: возвращает скорость ветра (магнитуда вектора)
    virtual metricType getWindSpeed( Eigen::Vector3<metricType> r, metricType t) const = 0;

    // Опционально: возвращает направление ветра (например, азимут и угол возвышения)
    virtual std::pair<metricType, metricType> getWindDirection( Eigen::Vector3<metricType> r, metricType t) const = 0;
};




