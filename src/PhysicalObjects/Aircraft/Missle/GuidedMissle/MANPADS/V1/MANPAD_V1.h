/*
//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once
#include "../../../../AbstractAircraft.h"
#include "../../../../../../DynamicsSystem/ExtensionModels//Aerodinamics/AeroInput/AeroInput.h"
#include "../../../../../../utils/ObjInitParams.h"

//
// Created by 4NR_Operator_3 on 13.10.2025.
//

template<typename metricType>
class MANPAD_V1 final: public AbstractAircraft<metricType> {
public:
    // Принимаем параметры по значению для возможности перемещения
    explicit MANPAD_V1(
        std::shared_ptr<IDynamicsSystem<metricType>> sys,
        ObjInitParams<metricType> initial_params,  // По значению вместо shared_ptr
        std::shared_ptr<AeroInput<metricType>> aerodynamicParams)
        : AbstractAircraft<metricType>(std::move(sys), std::move(initial_params))
    {
        this->aerodynamicParams_ = std::move(aerodynamicParams);
    }

    // Делаем методы константными где это уместно
    Eigen::Vector3<metricType> getAerodynamicForces() const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    Eigen::Vector3<metricType> getAerodynamicMoments() const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    // === СИСТЕМЫ УПРАВЛЕНИЯ ===
    Eigen::Vector3<metricType> getControlSurfacesDeflection() const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    void setControlInputs(metricType elevator, metricType aileron, metricType rudder) override {
        // Сохраняем значения управления для последующего использования
        controlInputs_ = {elevator, aileron, rudder};
    }

    // === ДАТЧИКИ ===
    Eigen::Vector3<metricType> getGyroscopeAngularVelocity() const override {
        // Возвращаем текущую угловую скорость из снапшота
        return this->presentSnapshot_.getAngularVelocity();
    }

    Eigen::Vector3<metricType> getAccelerometerAngularAcceleration() const override {
        // Исправлено: акселерометр измеряет линейное ускорение, а не угловое
        // Возвращаем текущее линейное ускорение из снапшота
        return this->presentSnapshot_.getAcceleration();
    }

private:
    // Добавляем член для хранения текущих управляющих воздействий
    Eigen::Vector3<metricType> controlInputs_ = Eigen::Vector3<metricType>::Zero();
};
*/


