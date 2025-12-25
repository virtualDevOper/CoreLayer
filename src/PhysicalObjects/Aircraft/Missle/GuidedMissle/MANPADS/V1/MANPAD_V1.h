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
        std::unique_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr)
        : GuidedMissle<metricType, RocketAeroInput<metricType>>(
            std::move(sys),
            std::move(initial_params),
            std::move(aero_input),
            std::move(comp_interp_mgr)) {}

    // === АЭРОДИНАМИЧЕСКИЕ СИЛЫ ===
    Eigen::Vector3<metricType> getAerodynamicForces(metricType t) const override {
        const auto& state = this->getStateSnapshot();

        metricType velocity_mag = state.getVelocity().norm();
        if (velocity_mag < 1e-6) return Eigen::Vector3<metricType>::Zero();

        metricType alpha = computeAngleOfAttack(state);
        metricType mach = computeMachNumber(velocity_mag);
        metricType rho = computeAirDensity(state);

        metricType q_dynamic = 0.5 * rho * velocity_mag * velocity_mag;
        auto C = this->comp_interp_mgr_->getAerodynamicForceCoefficients(alpha, mach);

        return q_dynamic * reference_area_ * C;
    }

    // === АЭРОДИНАМИЧЕСКИЕ МОМЕНТЫ ===
    Eigen::Vector3<metricType> getAerodynamicMoments(metricType t) const override {
        const auto& state = this->getStateSnapshot();

        metricType velocity_mag = state.getVelocity().norm();
        if (velocity_mag < 1e-6) return Eigen::Vector3<metricType>::Zero();

        metricType alpha = computeAngleOfAttack(state);
        metricType mach = computeMachNumber(velocity_mag);
        metricType rho = computeAirDensity(state);
        const auto& angular_vel = state.getAngularVelocity();

        metricType q_dynamic = 0.5 * rho * velocity_mag * velocity_mag;
        metricType control_deflection = 0.0;

        Eigen::Vector3<metricType> moments = Eigen::Vector3<metricType>::Zero();

        // Момент крена (X)
        metricType xd_k = this->comp_interp_mgr_->getXdKAero(alpha, mach);
        metricType a_ck_otn = this->comp_interp_mgr_->getAckOtnAero(alpha, mach);
        moments(0) = q_dynamic * reference_area_ * reference_length_ *
                     (xd_k * angular_vel(0) + a_ck_otn * control_deflection);

        // Момент рыскания (Z)
        metricType cy_r = this->comp_interp_mgr_->getCyRAero(alpha, mach);
        metricType cy_k = this->comp_interp_mgr_->getCyKAero(alpha, mach);
        metricType cy_st = this->comp_interp_mgr_->getCyStAero(alpha, mach);
        moments(2) = q_dynamic * reference_area_ * reference_length_ *
                     (cy_r * angular_vel(2) + cy_k * angular_vel(0) + cy_st * control_deflection);

        return moments;
    }

    // === ДАТЧИКИ БИНС ===
    Eigen::Vector3<metricType> getGyroscopeAngularVelocity(metricType t) const override {
        return this->getStateSnapshot().getAngularVelocity();
    }

    Eigen::Vector3<metricType> getAccelerometerAcceleration(metricType t) const override {
        return Eigen::Vector3<metricType>::Zero();
    }

    // === АВТОПИЛОТ ===
    void autopilot(metricType t) override {
        // Законы управления
    }

    // === УСТАНОВКА ПАРАМЕТРОВ ===
    void setReferenceArea(metricType S) { reference_area_ = S; }
    void setReferenceLength(metricType L) { reference_length_ = L; }

private:
    // === ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===
    metricType computeAngleOfAttack(const ObjSnapshot<metricType>& state) const {
        const auto& V = state.getVelocity();
        return std::atan2(V(2), V(0));
    }

    metricType computeMachNumber(metricType velocity_mag) const {
        return velocity_mag / 340.0;
    }

    metricType computeAirDensity(const ObjSnapshot<metricType>& state) const {
        metricType z = state.getPosition()(2);
        metricType rho_0 = 1.225;
        metricType H = 8500.0;
        return rho_0 * std::exp(-z / H);
    }
};


