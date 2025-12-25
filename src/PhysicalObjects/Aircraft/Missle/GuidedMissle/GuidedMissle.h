//
// Created by 4NR_Operator_3 on 28.11.2025.
//

#pragma once
#include "../../AbstractAircraft.h"


/**
 * \brief Абстрактный класс управляемой ракеты/снаряда/БПЛА
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Добавляется БИНС(3 ДУС, 3 акселерометра), а также автопилот с вычислением
 * динамики полета на борту
 * Сильно дальше при доработке проекта, можно добавить астрокоррекцию, возможно, в дочерних классах
 * для подкласса ОТРК
 */

template <typename metricType, typename AeroInput>
class GuidedMissle : public AbstractAircraft<metricType, AeroInput> {
public:
    explicit GuidedMissle(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<AeroInput> aero_input,
        std::unique_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr)
        : AbstractAircraft<metricType, AeroInput>(
            std::move(sys),
            std::move(initial_params),
            std::move(aero_input),
            std::move(comp_interp_mgr)) {}

    virtual ~GuidedMissle() = default;

    // === ДАТЧИКИ БИНС ===
    virtual Eigen::Vector3<metricType> getGyroscopeAngularVelocity(metricType t) const = 0;
    virtual Eigen::Vector3<metricType> getAccelerometerAcceleration(metricType t) const = 0;

    // === АВТОПИЛОТ ===
    virtual void autopilot(metricType t) = 0;
};

































