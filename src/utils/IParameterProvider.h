//
// Created by refactoring on 29.01.2026.
//

#pragma once
#include "PCH.h"

/**
 * \brief Интерфейс для провайдера динамических параметров объекта.
 *
 * \tparam metricType Тип данных для метрических величин.
 *
 * \details Абстракция для получения изменяющихся во времени параметров
 * физических объектов: массы, моментов инерции и тяги. Позволяет
 * разорвать циклические зависимости между системами динамики и
 * объектами, применяя принцип инверсии зависимостей (DIP).
 * Поддерживает интерполяцию параметров по времени.
 */
template<typename metricType>
class IParameterProvider {
public:
    virtual ~IParameterProvider() = default;

    // Получение массы в момент времени t
    [[nodiscard]] virtual metricType getMass(metricType t) const = 0;

    // Получение моментов инерции в момент времени t
    [[nodiscard]] virtual Eigen::Vector3<metricType> getInertia(metricType t) const = 0;

    // Получение вектора тяги в момент времени t
    [[nodiscard]] virtual Eigen::Vector3<metricType> getThrust(metricType t) const = 0;

    [[nodiscard]] virtual Eigen::Vector3<metricType> getCOM(metricType t) const = 0;
};
