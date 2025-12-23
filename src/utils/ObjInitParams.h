//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

/**
 * \brief Инициирующая структура для описания начальных параметров любого объекта
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \detail Вот тут не уверен насчет ускорений, мб они и не нужны, по идее я смогу их вычислить сам
 */


#pragma once

template<typename metricType>
struct ObjInitParams {
    ObjInitParams()
        : position(Eigen::Vector3<metricType>::Zero())
        , velocity(Eigen::Vector3<metricType>::Zero())
        , eulerAngles(Eigen::Vector3<metricType>::Zero())
        , angularVelocity(Eigen::Vector3<metricType>::Zero())
        , totalForce(Eigen::Vector3<metricType>::Zero())
        , totalMoment(Eigen::Vector3<metricType>::Zero())
        , mass(0.0)
        , inertia(Eigen::Vector3<metricType>::Zero()) {}

    Eigen::Vector3<metricType> position;
    Eigen::Vector3<metricType> velocity;
    Eigen::Vector3<metricType> eulerAngles;
    Eigen::Vector3<metricType> angularVelocity;
    Eigen::Vector3<metricType> totalForce;
    Eigen::Vector3<metricType> totalMoment;
    metricType mass;
    Eigen::Vector3<metricType> inertia;
};

