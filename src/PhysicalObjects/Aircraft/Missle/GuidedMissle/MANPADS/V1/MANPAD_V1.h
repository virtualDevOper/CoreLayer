//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once


#include "../../GuidedMissle.h"
#include "../../../../../../DynamicsSystem/ExtensionModels//Aerodinamics/AeroInput/RocketAeroInput.h"
#include "../../../../../../utils/ObjInitParams.h"

/**
 * \brief Класс реализации ЗУР_1_версия
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Я не вижу смысла разделять ракеты по классам (ЗУР ПТРК ОТРК).
 * Считаю, что все ракеты одинаковы и отличаются наличием дополнительной аппаратуры на борту.
 * То есть ОТР = ЗУР, просто в ОТР есть астрокоррекция и другие ништяки
 * Данный класс реализует ЗУР с БИНС, с полной аэродинамикой
 */


//TODO
// === Реализовать выше по иерархии передачу БИНС, головки самонаведений
// чтобы тут работало измерение угловой скорости и ускорения с датчика,
// а также угловое рассогласование с целью
// ЧТО делать с управляющими поврехностями? Автопилот должен менять их поворот===

template<typename metricType>
class MANPAD_V1 final: public GuidedMissle<metricType,RocketAeroInput<metricType>> {
public:
    explicit MANPAD_V1(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<RocketAeroInput<metricType> > aero_input)
        : GuidedMissle<metricType,RocketAeroInput<metricType>>
            (std::move(sys), std::move(initial_params), std::move(aero_input))
    {}

    Eigen::Vector3<metricType> getAerodynamicForces() const override {
        return Eigen::Vector3<metricType>::Zero();
    }
    Eigen::Vector3<metricType> getAerodynamicMoments() const override {
        return Eigen::Vector3<metricType>::Zero();
    }
    Eigen::Vector3<metricType> getGyroscopeAngularVelocity() const override {
        // Возвращаем текущую угловую скорость гироскопов! а они будут просто брать
        // из снепшота и немного добавлять шумы и неточности
        return Eigen::Vector3<metricType>::Zero();
    }
    Eigen::Vector3<metricType> getAccelerometerAngularAcceleration() const override {
        // Возвращаем текущее ускорение из акселерометров! а они будут просто брать
        // из снепшота и немного добавлять шумы и неточности
        return Eigen::Vector3<metricType>::Zero();
    }
    void autopilot () const override {auto a = 0;}

};


