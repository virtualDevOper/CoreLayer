//
// Created by 4NR_Operator_3 on 12.01.2026.
//

#pragma once
#include "../../PCH.h"


// Создание системы ОДУ для ракеты с провайдером, который будет хранить в себе
// getMass(t) getInertia(t) getThrust(t) getAerodynamicForces(t) getAerodynamicMoments(t)


// эта тема знает про интерполятор(шаред птр на интерполятор закинуть
// при создании) но не знает ничего про систему оду(в систему оду она передается
// и используется но может передаваться куда угодно), эта будет при
// инициализации сразу иметь доступ к менеджеру интерполяторов

template <typename metricType>
class DynamicParametersProviderForFullRocketModel {
private:
    std::weak_ptr<ComponentInterpolationManager<metricType>> interpolation_manager_;

public:
    explicit DynamicParametersProviderForFullRocketModel(
        std::shared_ptr<ComponentInterpolationManager<metricType>> interpolation_manager)
        : interpolation_manager_(interpolation_manager){}
    metricType getMass(metricType t) {
        return interpolation_manager_->getMass(t);
    }
    Eigen::Vector3<metricType> getInertia(metricType t) const {
        return interpolation_manager_->getInertia(t);
    }

    Eigen::Vector3<metricType> getInertia(metricType t) const {
        return interpolation_manager_->getInertia(t);
    }
}