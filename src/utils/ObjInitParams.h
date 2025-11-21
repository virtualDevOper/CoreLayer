/*
//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once
#include "../../PCH.h"

/**
 * \brief Инициирующая структура для описания начальных параметров любого объекта
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \detail Вот тут не уверен насчет ускорений, мб они и не нужны, по идее я смогу их вычислить сам
 #1#

//TODO
// === мб удалишь ускорения

template<typename metricType>
struct  ObjInitParams {
    Eigen::Vector3<metricType> position = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> velocity = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> acceleration = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> eulerAngles = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> angularVelocity = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> angularAcceleration = Eigen::Matrix<metricType, 3, 1>::Zero();
    metricType mass = 0;
    Eigen::Vector3<metricType> inertia = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> centerOfMass  = Eigen::Matrix<metricType, 3, 1>::Zero() ;
    Eigen::Vector3<metricType> totalForce  = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> totalMoment  = Eigen::Matrix<metricType, 3, 1>::Zero();
};
*/



