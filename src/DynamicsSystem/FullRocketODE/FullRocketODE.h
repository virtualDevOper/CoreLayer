//
// Created by 4NR_Operator_3 on 25.12.2025
// CORRECT version based on PDF mathematical model
// REFACTORED: Smart pointers everywhere
//

#pragma once
#include "../IDynamicsSystem.h"
#include "../../utils/ConeDirection/ConeDirection.h"



/**
 * \brief Полная система ОДУ для летательного аппарата (ракеты в данном случае)
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details СООТВЕТСТВУЕТ МАТЕМАТИЧЕСКОЙ МОДЕЛИ  из диплома Ильи Берулавы 1-2025:
 *
 * ВХОДНЫЕ ДАННЫЕ (ObjSnapshot в земной СК):
 * - position = (x_e, y_e, z_e) [м]
 * - velocity = (v_xe, v_ye, v_ze) [м/с]
 * - eulerAngles = (θ, Ψ, γ) [рад]
 * - angularVelocity = (ω_x, ω_y, ω_z) [рад/с] в связной СК(вот тут хз)!
 *
 * ВЫХОДНЫЕ ДАННЫЕ (производные в соответствующих СК):
 * - dPosition/dt = (v_xe, v_ye, v_ze)
 * - dVelocity/dt = (dv_xe/dt, dv_ye/dt, dv_ze/dt)
 * - dEulerAngles/dt = (dθ/dt, dΨ/dt, dγ/dt)
 * - dAngularVelocity/dt = (dω_x/dt, dω_y/dt, dω_z/dt) в связной СК(вот тут хз)
 */

template <typename metricType>
class FullRocketODE final : public IDynamicsSystem<metricType> {
private:
    std::shared_ptr<AbstractAircraft<metricType, RocketAeroInput<metricType>>> aircraft_;

public:
    explicit FullRocketODE(
        std::shared_ptr<AbstractAircraft<metricType, RocketAeroInput<metricType>>> aircraft)
        : aircraft_(std::move(aircraft)) {
        if (!aircraft_) {
            throw std::invalid_argument("Указатель на ЛА не может быть null");
        }
    }

    std::string get_description() override {
        return "Полная система ОДУ для ЛА (модель из диплома Ильи Берулавы 1-2025)";
    }

    /**
     * \brief Вычисляет производные состояния
     *
     * В соответствии с PDF:
     * - dv_x/dt = -ω_y·v_z + ω_z·v_y + F_sum_x(t) / m(t)
     * - dv_y/dt = -ω_z·v_x + ω_x·v_z + F_sum_y(t) / m(t)
     * - dv_z/dt = -ω_x·v_y + ω_y·v_x + F_sum_z(t) / m(t)
     *
     * - dω_x/dt = (M_sum_x - (I_z - I_y)·ω_y·ω_z) / I_x
     * - dω_y/dt = (M_sum_y - (I_x - I_z)·ω_x·ω_z) / I_y
     * - dω_z/dt = (M_sum_z - (I_y - I_x)·ω_x·ω_y) / I_z
     *
     * - dΨ/dt = (ω_y·cos(γ) - ω_z·sin(γ)) / cos(θ)
     * - dγ/dt = ω_x - tg(θ)·(ω_y·cos(γ) - ω_z·sin(γ))
     * - dθ/dt = ω_y·sin(γ) + ω_z·cos(γ)  [ было ω_z + ω_z, иишка исправила, а я в матеше не селен]
     *
     * - dx_e/dt = v_xe
     * - dy_e/dt = v_ye
     * - dz_e/dt = v_ze
     */
    std::unique_ptr<ObjSnapshot<metricType>> get_rhs_derivatives(
        const ObjSnapshot<metricType>& previous_state,
        metricType t) override {

        auto derivatives = compute_complete_solution(previous_state, t);

        return ObjSnapshot<metricType>::createBuilder()
            .setPosition(derivatives->dposition_dt)
            .setVelocity(derivatives->dvelocity_dt)
            .setEulerAngles(derivatives->deuler_dt)
            .setAngularVelocity(derivatives->dangular_velocity_dt)
            .buildUnique();
    }

private:
    struct Derivatives {
        Eigen::Vector3<metricType> dposition_dt;           // dx_e/dt, dy_e/dt, dz_e/dt
        Eigen::Vector3<metricType> dvelocity_dt;           // dv_xe/dt, dv_ye/dt, dv_ze/dt
        Eigen::Vector3<metricType> deuler_dt;              // dθ/dt, dΨ/dt, dγ/dt
        Eigen::Vector3<metricType> dangular_velocity_dt;   // dω_x/dt, dω_y/dt, dω_z/dt
    };

    std::unique_ptr<Derivatives> compute_complete_solution(
        const ObjSnapshot<metricType>& state, metricType t) {

        auto derivatives = std::make_unique<Derivatives>();

        // === ШАГ 1: ПОЛУЧИТЬ ПАРАМЕТРЫ ===
        metricType mass = aircraft_->getMass(t);
        auto inertia = aircraft_->getInertia(t);  // (I_x, I_y, I_z)

        // === ШАГ 2: ТРАНСФОРМИРОВАТЬ СКОРОСТЬ ИЗ ЗЕМНОЙ В СВЯЗНУЮ СК ===
        const auto& V_earth = state.getVelocity();  // (v_xe, v_ye, v_ze)
        const auto& euler = state.getEulerAngles();  // (θ, Ψ, γ)

        auto V_body = transformVelocityEarthToBody(V_earth, euler);
        // Теперь V_body = (v_x, v_y, v_z)

        // === ШАГ 3: ПОЛУЧИТЬ УГЛОВУЮ СКОРОСТЬ (уже в связной СК) ===
        const auto& omega = state.getAngularVelocity();  // (ω_x, ω_y, ω_z)

        // === ШАГ 4: ВЫЧИСЛИТЬ АЭРОДИНАМИЧЕСКИЕ УГЛЫ ===
        metricType alpha_p = computeSpaceAngleOfAttack(V_body);
        metricType phi_p = computeAerodynamicRollAngle(V_body);
        metricType mach = computeMachNumber(V_body.norm());
        metricType rho = computeAirDensity(state);

        // === ШАГ 5: ПОЛУЧИТЬ АЭРОДИНАМИЧЕСКИЕ СИЛЫ И МОМЕНТЫ ===
        // Силы и моменты возвращаются в связной СК!
        auto aero_forces_body = aircraft_->getAerodynamicForces(t);
        auto aero_moments_body = aircraft_->getAerodynamicMoments(t);

        // === ШАГ 6: ПОЛУЧИТЬ ТЯГУ (уже в связной СК) ===
        auto thrust_body = aircraft_->getThrust(t);

        // === ШАГ 7: СОСТАВИТЬ СУММАРНЫЕ СИЛЫ В СВЯЗНОЙ СК ===
        auto F_sum_body = computeTotalForces(
            aero_forces_body, thrust_body, mass, euler);

        // === ШАГ 8: ВЫЧИСЛИТЬ ПРОИЗВОДНЫЕ СКОРОСТИ В СВЯЗНОЙ СК ===
        auto dV_body_dt = computeLinearAccelerationBody(
            V_body, omega, F_sum_body, mass);

        // === ШАГ 9: ТРАНСФОРМИРОВАТЬ ПРОИЗВОДНУЮ СКОРОСТИ В ЗЕМНУЮ СК ===
        derivatives->dvelocity_dt = transformAccelerationBodyToEarth(
            dV_body_dt, V_body, omega, euler);

        // === ШАГ 10: ВЫЧИСЛИТЬ ПРОИЗВОДНЫЕ УГЛОВ ЭЙЛЕРА ===
        derivatives->deuler_dt = computeEulerAnglesDerivatives(euler, omega);

        // === ШАГ 11: ВЫЧИСЛИТЬ ПРОИЗВОДНЫЕ УГЛОВОЙ СКОРОСТИ В СВЯЗНОЙ СК ===
        derivatives->dangular_velocity_dt = computeAngularAccelerationBody(
            aero_moments_body, inertia, omega);

        // === ШАГ 12: СОБРАТЬ РЕЗУЛЬТАТЫ ===
        derivatives->dposition_dt = state.getVelocity();

        return derivatives;
    }

    // ================= ТРАНСФОРМАЦИИ МЕЖДУ СК =================

    /**
     * \brief Трансформирует скорость из земной СК в связную СК
     * V_body = A^T · V_earth
     */
    Eigen::Vector3<metricType> transformVelocityEarthToBody(
        const Eigen::Vector3<metricType>& V_earth,
        const Eigen::Vector3<metricType>& euler) const {

        std::array<metricType, 3> V_earth_array = {V_earth(0), V_earth(1), V_earth(2)};

        auto transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
            euler(0), euler(1), euler(2), V_earth_array);

        auto result = transform->result_getter();
        return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
    }

    /**
     * \brief Трансформирует ускорение из связной СК в земную СК
     * a_earth = A · a_body
     */
    Eigen::Vector3<metricType> transformAccelerationBodyToEarth(
        const Eigen::Vector3<metricType>& a_body,
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& euler) const {

        std::array<metricType, 3> a_body_array = {a_body(0), a_body(1), a_body(2)};

        auto transform = std::make_unique<Svyaz_to_zemn_Direction<metricType>>(
            euler(0), euler(1), euler(2), a_body_array);

        auto result = transform->result_getter();
        return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
    }

    // ================= ВЫЧИСЛЕНИЯ =================

    /**
     * \brief Пространственный угол атаки
     * α_p = arcsin(√(v_y² + v_z²) / √(v_x² + v_y² + v_z²))
     */
    metricType computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const {
        metricType v_x = V_body(0);
        metricType v_y = V_body(1);
        metricType v_z = V_body(2);

        metricType numerator = std::sqrt(v_y*v_y + v_z*v_z);
        metricType denominator = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);

        if (denominator < 1e-6) return 0.0;
        return std::asin(numerator / denominator);
    }

    /**
     * \brief Аэродинамический угол крена
     * sin(φ_p) = v_z / √(v_y² + v_z²)
     * cos(φ_p) = -v_y / √(v_y² + v_z²)
     *
     * Если v_y² + v_z² = 0: sin(φ_p) = 0, cos(φ_p) = 1
     */
    metricType computeAerodynamicRollAngle(const Eigen::Vector3<metricType>& V_body) const {
        metricType v_y = V_body(1);
        metricType v_z = V_body(2);

        metricType denominator = std::sqrt(v_y*v_y + v_z*v_z);

        if (denominator < 1e-6) return 0.0;
        return std::atan2(v_z, -v_y);
    }

    metricType computeMachNumber(metricType V_magnitude) const {
        return V_magnitude / 340.0;  // [м/с] скорость звука
    }

    metricType computeAirDensity(const ObjSnapshot<metricType>& state) const {
        metricType z = state.getPosition()(2);
        metricType rho_0 = 1.225;  // [кг/м³] на уровне моря
        metricType H = 8500.0;     // [м] масштабная высота
        return rho_0 * std::exp(-z / H);
    }

    /**
     * \brief Суммарные силы в связной СК (включая гравитацию)
     *
     * F_sum = [R_t - X_ad, Y_ad + R_s, Z_ad + R_z] + A^T·[0, -m·g, 0]
     */
    Eigen::Vector3<metricType> computeTotalForces(
        const Eigen::Vector3<metricType>& F_aero,
        const Eigen::Vector3<metricType>& F_thrust,
        metricType mass,
        const Eigen::Vector3<metricType>& euler) const {

        // Гравитация в земной СК: [0, -m·g, 0]
        std::array<metricType, 3> gravity_array = {0, -mass * 9.81, 0};

        auto gravity_transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
            euler(0), euler(1), euler(2), gravity_array);

        auto gravity_body_array = gravity_transform->result_getter();
        Eigen::Vector3<metricType> gravity_body(
            gravity_body_array[0], gravity_body_array[1], gravity_body_array[2]);

        return F_aero + F_thrust + gravity_body;
    }

    /**
     * \brief Линейное ускорение в связной СК
     *
     * dv_x/dt = -ω_y·v_z + ω_z·v_y + F_sum_x / m
     * dv_y/dt = -ω_z·v_x + ω_x·v_z + F_sum_y / m
     * dv_z/dt = -ω_x·v_y + ω_y·v_x + F_sum_z / m
     */
    Eigen::Vector3<metricType> computeLinearAccelerationBody(
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& F_sum,
        metricType mass) const {

        metricType v_x = V_body(0), v_y = V_body(1), v_z = V_body(2);
        metricType w_x = omega(0), w_y = omega(1), w_z = omega(2);

        Eigen::Vector3<metricType> dV_dt;
        dV_dt(0) = -w_y * v_z + w_z * v_y + F_sum(0) / mass;
        dV_dt(1) = -w_z * v_x + w_x * v_z + F_sum(1) / mass;
        dV_dt(2) = -w_x * v_y + w_y * v_x + F_sum(2) / mass;

        return dV_dt;
    }

    /**
     * \brief Производные углов Эйлера
     *
     * dΨ/dt = (ω_y·cos(γ) - ω_z·sin(γ)) / cos(θ)
     * dγ/dt = ω_x - tg(θ)·(ω_y·cos(γ) - ω_z·sin(γ))
     * dθ/dt = ω_y·sin(γ) + ω_z·cos(γ)   [ИСПРАВЛЕНО]
     */
    Eigen::Vector3<metricType> computeEulerAnglesDerivatives(
        const Eigen::Vector3<metricType>& euler,
        const Eigen::Vector3<metricType>& omega) const {

        metricType theta = euler(0);  // тангаж
        metricType psi = euler(1);    // рыскание
        metricType gamma = euler(2);  // крен

        metricType w_x = omega(0);
        metricType w_y = omega(1);
        metricType w_z = omega(2);

        metricType cos_theta = std::cos(theta);
        metricType sin_theta = std::sin(theta);
        metricType cos_gamma = std::cos(gamma);
        metricType sin_gamma = std::sin(gamma);
        metricType tan_theta = std::tan(theta);

        Eigen::Vector3<metricType> dEuler_dt;

        // dΨ/dt (рыскание)
        if (std::abs(cos_theta) > 1e-6) {
            dEuler_dt(1) = (w_y * cos_gamma - w_z * sin_gamma) / cos_theta;
        } else {
            dEuler_dt(1) = 0.0;  // Сингулярность при θ = ±π/2
        }

        // dγ/dt (крен)
        dEuler_dt(2) = w_x - tan_theta * (w_y * cos_gamma - w_z * sin_gamma);

        // dθ/dt (тангаж) — ИСПРАВЛЕНО!
        dEuler_dt(0) = w_y * sin_gamma + w_z * cos_gamma;

        return dEuler_dt;
    }

    /**
     * \brief Угловое ускорение в связной СК
     *
     * M_sum_x = I_x·dω_x/dt + (I_z - I_y)·ω_y·ω_z
     * => dω_x/dt = (M_sum_x - (I_z - I_y)·ω_y·ω_z) / I_x
     */
    Eigen::Vector3<metricType> computeAngularAccelerationBody(
        const Eigen::Vector3<metricType>& M_sum,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& omega) const {

        metricType I_x = inertia(0);
        metricType I_y = inertia(1);
        metricType I_z = inertia(2);

        metricType w_x = omega(0);
        metricType w_y = omega(1);
        metricType w_z = omega(2);

        metricType M_x = M_sum(0);
        metricType M_y = M_sum(1);
        metricType M_z = M_sum(2);

        Eigen::Vector3<metricType> dOmega_dt;

        dOmega_dt(0) = (M_x - (I_z - I_y) * w_y * w_z) / I_x;
        dOmega_dt(1) = (M_y - (I_x - I_z) * w_x * w_z) / I_y;
        dOmega_dt(2) = (M_z - (I_y - I_x) * w_x * w_y) / I_z;

        return dOmega_dt;
    }
};
