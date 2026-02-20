//
// Created by 4NR_Operator_3 on 12.01.2026.
//

#pragma once
#include "PCH.h"
#include "Interpolation/ComponentInterpolationManager.h"

// Создание системы ОДУ для ракеты с провайдером, который будет хранить в себе
// getMass(t) getInertia(t) getThrust(t)
// Аэродинамика теперь обрабатывается отдельной библиотекой aero_simpi


// эта тема знает про интерполятор(шаред птр на интерполятор закинуть
// при создании) но не знает ничего про систему оду(в систему оду она передается
// и используется но может передаваться куда угодно), эта будет при
// инициализации сразу иметь доступ к менеджеру интерполяторов


//TODO
// == Убери заглушки!! и тут можно убрать использование простой модели ад автоматически


template <typename metricType>
class DynamicParametersProviderForFullRocketModel {
private:
    std::weak_ptr<ComponentInterpolationManager<metricType>> interpolation_manager_;

public:
    explicit DynamicParametersProviderForFullRocketModel(
        std::shared_ptr<ComponentInterpolationManager<metricType>> interpolation_manager)
    : interpolation_manager_(interpolation_manager) {}

    metricType getMass(metricType t) const {
        auto manager = interpolation_manager_.lock();
        if (!manager) throw std::runtime_error("Interpolation manager expired");
        return manager->getMass(t);
    }

    Eigen::Vector3<metricType> getInertia(metricType t) const {
        auto manager = interpolation_manager_.lock();
        if (!manager) throw std::runtime_error("Interpolation manager expired");
        return manager->getInertia(t);
    }

    Eigen::Vector3<metricType> getThrust(metricType t) const {
        auto manager = interpolation_manager_.lock();
        if (!manager) throw std::runtime_error("Interpolation manager expired");
        return manager->getThrust(t);
    }

    [[nodiscard]] Eigen::Vector3<metricType> getCOM(metricType t) const {
        auto manager = interpolation_manager_.lock();
        if (!manager) throw std::runtime_error("Interpolation manager expired");
        return manager->getCOM(t);
    }
};