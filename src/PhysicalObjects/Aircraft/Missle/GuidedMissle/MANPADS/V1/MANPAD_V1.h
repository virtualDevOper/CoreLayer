//
// Created by 4NR_Operator_3 on 13.10.2025.
//
#pragma once
#include "../../GuidedMissle.h"
#include "../../../../../../DynamicsSystem/ExtensionModels//Aerodinamics/AeroInput/RocketAeroInput.h"
#include "../../../../../../utils/ObjInitParams.h"
#include "../../../../../../DynamicsSystem/ExtensionModels/Aerodinamics/IAeroModel.h"
/**
* \brief Класс реализации ЗУР_1_версия
*
* \tparam metricType Тип данных для метрических величин
*
* \details Полная система с:
* - Интерполяцией массы, инерции, тяги
* - Расчётом аэродинамических сил и моментов
* - БИНС и автопилотом
*/
//TODO
// === Реализовать выше по иерархии передачу БИНС, головки самонаведений
// чтобы тут работало измерение угловой скорости и ускорения с датчика,
// а также угловое рассогласование с целью
// ЧТО делать с управляющими поверхностями? Автопилот должен менять их поворот===

template<typename metricType>
class MANPAD_V1 final : public GuidedMissle<metricType, RocketAeroInput<metricType>> {
public:
    explicit MANPAD_V1(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<RocketAeroInput<metricType>> aero_input,
        std::shared_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr,
        std::shared_ptr<IAeroModel<metricType>> aero_model)
        : GuidedMissle<metricType, RocketAeroInput<metricType>>(
            std::move(sys),
            std::move(initial_params),
            std::move(aero_input),
            comp_interp_mgr),
          aero_model_(std::move(aero_model))
    {
        if (!aero_model_) {
            throw std::invalid_argument("MANPAD_V1: aero_model cannot be null");
        }
    }

    Eigen::Vector3<metricType> getGyroscopeAngularVelocity(metricType t) const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    Eigen::Vector3<metricType> getAccelerometerAcceleration(metricType t) const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    void autopilot(metricType t) override {
        // TODO: реализация автопилота
    }

private:
    std::shared_ptr<IAeroModel<metricType>> aero_model_;
};