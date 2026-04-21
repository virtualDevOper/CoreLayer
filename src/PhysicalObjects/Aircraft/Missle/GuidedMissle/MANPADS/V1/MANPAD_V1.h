//
// Created by 4NR_Operator_3 on 13.10.2025.
//
#pragma once
#include "../../../../../AbstractObject.h"
#include "../../../../../../utils/ObjInitParams.h"
#include "../../../../../../utils/ParameterProvider/IParameterProvider.h"

/**
* \brief Класс реализации ЗУР_1_версия (MANPAD - Man-Portable Air-Defense System)
*
* \tparam metricType Тип данных для метрических величин
*
* \details Полная система с:
* - Интерполяцией массы, инерции, тяги
* - Расчётом аэродинамических сил и моментов (через aero_simpi в FullRocketODE)
* - БИНС и автопилотом. Да - ДА, верим_)
*
* Если что, БИНС - Бесплатформенная Инерциальная Навигационная Система. По факту даже в месте, где я до этого работал, шли споры, что в нее реально входит.
* В моем понимании это 3 акселерометра и 3 гироскопа. Вычислитель находится отдельно от этого всего. Вычислитель же в данном случае нужен для решения ОДУ с числами, полученными с датчиков(то есть с ошибками).
* В общем, если интересно - спроси нейросеть про эту тему. В дальнейшем нужно бы реализовать отдельную модель автопилота (разных версий) и модель датчиков акселерометров и гироскопов со своими запаздываниями, шумами и тд.
* Аэродинамика обрабатывается в FullRocketODE через библиотеку aero_simpi,
*/

template<typename metricType>
class MANPAD_V1 final : public AbstractObject<metricType> {
public:
    explicit MANPAD_V1(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::shared_ptr<IParameterProvider<metricType>> param_provider)
        : AbstractObject<metricType>(std::move(sys), std::move(initial_params)),
          param_provider_(param_provider)
    {
        if (!param_provider_) {throw std::invalid_argument("ParameterProvider is not available");}
    }

    // === ДАТЧИКИ БИНС ===
    Eigen::Vector3<metricType> getGyroscopeAngularVelocity(metricType t) const {
        // TODO: Добавить шум датчика
        return this->getStateSnapshot().getAngularVelocity();
    }

    Eigen::Vector3<metricType> getAccelerometerAcceleration(metricType t) const {
        // TODO: Добавить шум датчика и вычислить ускорение из производной скорости
        return Eigen::Vector3<metricType>::Zero();
    }

    // === АВТОПИЛОТ ===
    void autopilot(metricType t) {
        // TODO: реализация автопилота
        // Здесь будет логика управления АОУ и ГОУ, реализован ПИД регулятор ну и все в духе ТЕСЛА))
    }

protected:
    std::shared_ptr<IParameterProvider<metricType>> param_provider_;

    [[nodiscard]] std::shared_ptr<IParameterProvider<metricType>> getParameterProvider() const {
        auto provider = param_provider_.lock();
        if (!provider) {
            throw std::runtime_error("ParameterProvider is no longer available");
        }
        return provider;
    }
};