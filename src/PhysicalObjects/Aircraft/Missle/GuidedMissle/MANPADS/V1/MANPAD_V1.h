//
// Created by 4NR_Operator_3 on 13.10.2025.
//

#pragma once

#include "../../GuidedMissle.h"
#include "../../../../../../DynamicsSystem/ExtensionModels//Aerodinamics/AeroInput/RocketAeroInput.h"
#include "../../../../../../utils/ObjInitParams.h"
#include "../../../../../../DynamicsSystem/ExtensionModels/Aerodinamics/FullAeroModel/FullAeroModel.h"


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
// ЧТО делать с управляющими поврехностями? Автопилот должен менять их поворот===


template<typename metricType>
class MANPAD_V1 final : public GuidedMissle<metricType, RocketAeroInput<metricType>> {
public:
    explicit MANPAD_V1(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<RocketAeroInput<metricType>> aero_input,
        std::shared_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr)
        : GuidedMissle<metricType, RocketAeroInput<metricType>>(
              std::move(sys),
              std::move(initial_params),
              std::move(aero_input),
              comp_interp_mgr),
          aero_model_(std::make_unique<FullAeroModel<metricType>>(
              this->getAeroInput(),
              comp_interp_mgr))
    {
        if (!aero_model_) {
            throw std::runtime_error("Failed to create aerodynamic model");
        }
    }

    Eigen::Vector3<metricType> getGyroscopeAngularVelocity(metricType t) const override {
        try {
            // TODO: реализовать с использованием датчиков
            return Eigen::Vector3<metricType>::Zero();
        } catch (const std::exception& e) {
            throw std::runtime_error("Gyroscope error at time " + std::to_string(t) + ": " + e.what());
        }
    }

    Eigen::Vector3<metricType> getAccelerometerAcceleration(metricType t) const override {
        try {
            // TODO: реализовать с использованием датчиков
            return Eigen::Vector3<metricType>::Zero();
        } catch (const std::exception& e) {
            throw std::runtime_error("Accelerometer error at time " + std::to_string(t) + ": " + e.what());
        }
    }

    void autopilot(metricType t) override {
        try {
            auto a = 10;

        } catch (const std::exception& e) {
            throw std::runtime_error("Autopilot error at time " + std::to_string(t) + ": " + e.what());
        }
    }

private:
    std::unique_ptr<FullAeroModel<metricType>> aero_model_;

    std::vector<metricType> getRudderDeflections(metricType t) const {
        try {
            std::vector<metricType> deflections(4, 0.0); // 4 руля
            // TODO: Реализовать логику расчета углов отклонения
            return deflections;
        } catch (const std::exception& e) {
            throw std::runtime_error("Rudder deflection error at time " + std::to_string(t) + ": " + e.what());
        }
    }
};

