/*
//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../AbstractObject.h"

template <typename metricType>
class AbstractAircraft : public AbstractObject<metricType> {
public:
    // Используем perfect forwarding для параметров
    template<typename... Args>
    explicit AbstractAircraft(Args&&... args)
        : AbstractObject<metricType>(std::forward<Args>(args)...) {};

    /*
    нужно продумать интерфейс абстрактного объекта, который в свою очередь имеет свой id и иметь указатель на систему оду, по которой он будет решаться
    #1#

    // === АЭРОДИНАМИЧЕСКИЕ ХАРАКТЕРИСТИКИ ===
    // Возвращаем по значению для небольших объектов (Eigen::Vector3 обычно эффективен)
    virtual Eigen::Vector3<metricType> getAerodynamicForces() const = 0;
    virtual Eigen::Vector3<metricType> getAerodynamicMoments() const = 0;

    // === СИСТЕМЫ УПРАВЛЕНИЯ ===
    virtual Eigen::Vector3<metricType> getControlSurfacesDeflection() const = 0;

    // Принимаем простые типы по значению
    virtual void setControlInputs(metricType elevator, metricType aileron, metricType rudder) = 0;

    // === ДАТЧИКИ(ПО ИДЕЕ У ВСЕХ ЛА ДОЛЖНЫ БЫТЬ) ===
    virtual Eigen::Vector3<metricType> getGyroscopeAngularVelocity() const = 0;
    virtual Eigen::Vector3<metricType> getAccelerometerAngularAcceleration() const = 0;

    // Вот тут внутри ЛА в любом случае будет решаться ОДУ, так что необходим будет сохранятель, то есть задача ОДУ будет решаться в 2 местах сразу

    // === БОРТОВАЯ АППАРАТУРА и автопилот ===
};
*/
