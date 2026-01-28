//
// Created by 4NR_Operator_3 on 25.12.2025
//
#pragma once
#include "../IDynamicsSystem.h"
#include "../../utils/ConeDirection/ConeDirection.h"
#include "../../utils/DynamicParametersProviderForFullRocketModel.h"

/**
 * \brief Полная система ОДУ для твёрдого тела (ракеты) в 6 степенях свободы.
 *
 * \tparam metricType Тип данных для метрических величин.
 *
 * \details Инкапсулирует правые части уравнений движения:
 * трансформации между земной и связанной СК, расчёт аэродинамических сил,
 * моментов, гравитации и инерционных эффектов. Используется интегратором
 * RungeKutta4Solver для пошагового расчёта траектории.
 */
template <typename metricType>
class FullRocketODE final : public IDynamicsSystem<metricType> {
private:
    std::weak_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider_;
    std::weak_ptr<AbstractWorldModel<metricType>> world_;
    // Буферы для сохранения последних вычисленных сил и моментов
    mutable Eigen::Vector3<metricType> last_computed_forces_;
    mutable Eigen::Vector3<metricType> last_computed_moments_;

public:
    explicit FullRocketODE(
        const std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
        const std::shared_ptr<AbstractWorldModel<metricType>> world
    )
        : params_provider_(params_provider),
          world_(world),
          last_computed_forces_(Eigen::Vector3<metricType>::Zero()),
          last_computed_moments_(Eigen::Vector3<metricType>::Zero())
    {
        if (params_provider_.expired()) {
            throw std::invalid_argument("Поставщик параметров не может быть null");
        }
    }

    std::string get_description() override {
        return "Полная система ОДУ для ЛА (модель из диплома Ралии Берулавы 1-2025)";
    }

    /**
     * \brief Вычисляет производные состояния (ТОЛЬКО кинематику!)
     */
    std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) override {
        // === ШАГ 1: ПОЛУЧИТЬ ПАРАМЕТРЫ ===
        auto params_provider = params_provider_.lock();
        if (!params_provider) {
            throw std::runtime_error("DynamicParametersProvider has been destroyed");
        }
        metricType mass = params_provider->getMass(t);
        auto inertia = params_provider->getInertia(t);

        // === ШАГ 2: ТРАНСФОРМИРОВАТЬ СКОРОСТЬ ИЗ ЗЕМНОЙ В СВЯЗАННУЮ СК ===
        const auto& V_earth = kinematics.getVelocity();
        const auto& euler = kinematics.getEulerAngles();
        auto V_body = transformVelocityEarthToBody(V_earth, euler);

        // === ШАГ 3: ПОЛУЧИТЬ УГЛОВУЮ СКОРОСТЬ (уже в связанной СК) ===
        const auto& omega = kinematics.getAngularVelocity();

        // === ШАГ 4: ВЫЧИСЛИТЬ АЭРОДИНАМИЧЕСКИЕ СИЛЫ И МОМЕНТЫ ===
        metricType alpha_p = computeSpaceAngleOfAttack(V_body);
        metricType phi_p = computeAerodynamicRollAngle(V_body);
        metricType mach = computeMachNumber(V_body.norm(), Eigen::Vector3<metricType>::Zero()); // TODO: передать реальную позицию
        metricType rho = computeAirDensity(Eigen::Vector3<metricType>::Zero()); // TODO: передать реальную позицию

        auto aero_model = params_provider->getAeroModel();
        auto aero_forces_body = aero_model->computeAerodynamicForces(V_body, rho, mach);
        auto aero_moments_body = aero_model->computeAerodynamicMoments(V_body, omega, rho, mach, euler);

        last_computed_forces_ = aero_forces_body;
        last_computed_moments_ = aero_moments_body;

        // === ШАГ 5: ПОЛУЧИТЬ ТЯГУ ===
        auto thrust_body = params_provider->getThrust(t);

        // === ШАГ 6: СУММАРНЫЕ СИЛЫ ===
        auto F_sum_body = computeTotalForces(aero_forces_body, thrust_body, mass, euler);
        last_computed_forces_ = F_sum_body;

        // === ШАГ 7: ВЫЧИСЛИТЬ ПРОИЗВОДНЫЕ ===
        // Линейное ускорение в связанной СК
        auto dV_body_dt = computeLinearAccelerationBody(V_body, omega, F_sum_body, mass);
        // Трансформация ускорения в земную СК
        auto dV_earth_dt = transformAccelerationBodyToEarth(dV_body_dt, V_body, omega, euler);
        // Производные углов Эйлера
        auto dEuler_dt = computeEulerAnglesDerivatives(euler, omega);
        // Угловое ускорение
        auto dOmega_dt = computeAngularAccelerationBody(aero_moments_body, inertia, omega);

        // === ШАГ 8: СОБРАТЬ РЕЗУЛЬТАТ (ТОЛЬКО КИНЕМАТИКА!) ===
        return std::make_unique<KinematicState<metricType>>(
            KinematicState<metricType>::createBuilder()
                .setPosition(kinematics.getVelocity())  // dx/dt = v
                .setVelocity(dV_earth_dt)               // dv/dt = вычисленное ускорение
                .setEulerAngles(dEuler_dt)              // dEuler/dt
                .setAngularVelocity(dOmega_dt)          // dOmega/dt
                .build()
        );
    }

    // Дополнительное обновление снапшота физическими параметрами на времени t
    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) const override {
        auto params_provider = params_provider_.lock();
        if (!params_provider) {
            throw std::runtime_error("DynamicParametersProvider expired in augmentSnapshot");
        }

        // Пересчёт сил и моментов из связанной СК в земную для логирования
        const auto& euler = kinematics.getEulerAngles();
        std::array<metricType, 3> F_body_arr = {
            last_computed_forces_.x(),
            last_computed_forces_.y(),
            last_computed_forces_.z()
        };
        auto F_earth_arr = TransformationFactory<metricType>::createBodyToEarthTransform(
            euler.x(), euler.y(), euler.z(), F_body_arr)->result_getter();
        Eigen::Vector3<metricType> F_earth(F_earth_arr[0], F_earth_arr[1], F_earth_arr[2]);

        std::array<metricType, 3> M_body_arr = {
            last_computed_moments_.x(),
            last_computed_moments_.y(),
            last_computed_moments_.z()
        };
        auto M_earth_arr = TransformationFactory<metricType>::createBodyToEarthTransform(
            euler.x(), euler.y(), euler.z(), M_body_arr)->result_getter();
        Eigen::Vector3<metricType> M_earth(M_earth_arr[0], M_earth_arr[1], M_earth_arr[2]);

        return ObjSnapshot<metricType>::createBuilder(kinematics)
            .setTime(t)
            .setMass(params_provider->getMass(t))
            .setInertia(params_provider->getInertia(t))
            .setTotalForce(last_computed_forces_)
            .setTotalMoment(last_computed_moments_)
            .setTotalForceEarth(F_earth)
            .setTotalMomentEarth(M_earth)
            .buildUnique();
    }

private:
    // ================= ТРАНСФОРМАЦИЯ МЕЖДУ СК =================
    Eigen::Vector3<metricType> transformVelocityEarthToBody(
        const Eigen::Vector3<metricType>& V_earth,
        const Eigen::Vector3<metricType>& euler) const
    {
        std::array<metricType, 3> V_earth_array = {V_earth(0), V_earth(1), V_earth(2)};
        auto transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
            euler(0), euler(1), euler(2), V_earth_array);
        auto result = transform->result_getter();
        return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
    }

    Eigen::Vector3<metricType> transformAccelerationBodyToEarth(
        const Eigen::Vector3<metricType>& a_body,
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& euler) const
    {
        std::array<metricType, 3> a_body_array = {a_body(0), a_body(1), a_body(2)};
        auto transform = std::make_unique<Svyaz_to_zemn_Direction<metricType>>(
            euler(0), euler(1), euler(2), a_body_array);
        auto result = transform->result_getter();
        return Eigen::Vector3<metricType>(result[0], result[1], result[2]);
    }

    // ================= ВЫЧИСЛЕНИЯ =================
    metricType computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const {
        metricType v_x = V_body(0);
        metricType v_y = V_body(1);
        metricType v_z = V_body(2);
        metricType numerator = std::sqrt(v_y*v_y + v_z*v_z);
        metricType denominator = std::sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
        if (denominator < 1e-6) return 0.0;
        return std::asin(numerator / denominator);
    }

    metricType computeAerodynamicRollAngle(const Eigen::Vector3<metricType>& V_body) const {
        metricType v_y = V_body(1);
        metricType v_z = V_body(2);
        metricType denominator = std::sqrt(v_y*v_y + v_z*v_z);
        if (denominator < 1e-6) return 0.0;
        return std::atan2(v_z, -v_y);
    }

    metricType computeMachNumber(metricType V_magnitude, const Eigen::Vector3<metricType>& position) const {
        auto world = world_.lock();
        if (!world || !world->getAtmosphericModel()) {
            throw std::runtime_error("World or atmospheric model not available");
        }
        return V_magnitude / world->getAtmosphericModel()->getSpeedOfSound(position);
    }

    metricType computeAirDensity(const Eigen::Vector3<metricType>& position) const {
        auto world = world_.lock();
        if (!world || !world->getAtmosphericModel()) {
            throw std::runtime_error("World or atmospheric model not available");
        }
        return world->getAtmosphericModel()->getDensity(position);
    }

    Eigen::Vector3<metricType> computeTotalForces(
        const Eigen::Vector3<metricType>& F_aero,
        const Eigen::Vector3<metricType>& F_thrust,
        metricType mass,
        const Eigen::Vector3<metricType>& euler) const
    {
        std::array<metricType, 3> gravity_array = {0, -mass * PhysicsConstants::g, 0};
        auto gravity_transform = std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
            euler(0), euler(1), euler(2), gravity_array);
        auto gravity_body_array = gravity_transform->result_getter();
        Eigen::Vector3<metricType> gravity_body(
            gravity_body_array[0], gravity_body_array[1], gravity_body_array[2]);
        return F_aero + F_thrust + gravity_body;
    }

    Eigen::Vector3<metricType> computeLinearAccelerationBody(
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& F_sum,
        metricType mass) const
    {
        metricType v_x = V_body(0), v_y = V_body(1), v_z = V_body(2);
        metricType w_x = omega(0), w_y = omega(1), w_z = omega(2);
        Eigen::Vector3<metricType> dV_dt;
        dV_dt(0) = -w_y * v_z + w_z * v_y + F_sum(0) / mass;
        dV_dt(1) = -w_z * v_x + w_x * v_z + F_sum(1) / mass;
        dV_dt(2) = -w_x * v_y + w_y * v_x + F_sum(2) / mass;
        return dV_dt;
    }

    Eigen::Vector3<metricType> computeEulerAnglesDerivatives(
        const Eigen::Vector3<metricType>& euler,
        const Eigen::Vector3<metricType>& omega) const
    {
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
        if (std::abs(cos_theta) > 1e-6) {
            dEuler_dt(1) = (w_y * cos_gamma - w_z * sin_gamma) / cos_theta;  // dPsi/dt
        } else {
            dEuler_dt(1) = 0.0;
        }
        dEuler_dt(2) = w_x - tan_theta * (w_y * cos_gamma - w_z * sin_gamma);  // dGamma/dt
        dEuler_dt(0) = w_y * sin_gamma + w_z * cos_gamma;  // dTheta/dt
        if (!std::isfinite(dEuler_dt(0)) || !std::isfinite(dEuler_dt(1)) || !std::isfinite(dEuler_dt(2))) {
            throw std::runtime_error("Non-finite values in Euler angles derivatives");
        }
        return dEuler_dt;
    }

    Eigen::Vector3<metricType> computeAngularAccelerationBody(
        const Eigen::Vector3<metricType>& M_sum,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& omega) const
    {
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