//
// Created by 4NR_Operator_3 on 12.01.2026.
//

#pragma once
#include "PCH.h"
#include "Interpolation/ComponentInterpolationManager.h"
#include "../DynamicsSystem/ExtensionModels/Aerodinamics/IAeroModel.h"
#include "../DynamicsSystem/ExtensionModels/Aerodinamics/SimpleAeroModel.h"

// Создание системы ОДУ для ракеты с провайдером, который будет хранить в себе
// getMass(t) getInertia(t) getThrust(t) getAerodynamicForces(t) getAerodynamicMoments(t)


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
    std::shared_ptr<IAeroModel<metricType>> aero_model_;

public:
    explicit DynamicParametersProviderForFullRocketModel(
        std::shared_ptr<ComponentInterpolationManager<metricType>> interpolation_manager,
        std::shared_ptr<IAeroModel<metricType>> aero_model = nullptr)
    : interpolation_manager_(interpolation_manager),
      aero_model_(aero_model ? aero_model : std::make_shared<SimpleAeroModel<metricType>>(0.05f)) {}

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
    
    /// \brief Доступ к текущей аэродинамической модели
    ///
    /// Вся физика аэродинамики инкапсулирована в IAeroModel и её реализациях.
    std::shared_ptr<IAeroModel<metricType>> getAeroModel() const {
        return aero_model_;
    }
};