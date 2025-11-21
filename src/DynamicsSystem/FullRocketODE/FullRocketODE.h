//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../../PCH.h"
#include "../IDynamicsSystem.h"


template <typename metricType>
class FullRocketODE final :public IDynamicsSystem<metricType> {
public:
    std::string get_description() override {
        return "Полная система ОДУ для ЛА, учитывающая динамику твердого тела,"
               " аэродинамические силы, тягу двигателя и гравитацию";
    }

    std::unique_ptr<ObjSnapshot<metricType>> get_rhs_derivatives(
        const ObjSnapshot<metricType>& previous_state,
        metricType t) override {

        auto [derivatives, interpolatedValues] = compute_complete_solution(previous_state, t);

        return ObjSnapshot<metricType>::createBuilder()
            .setPosition(previous_state.getPosition() + derivatives.position)
            .setVelocity(previous_state.getVelocity() + derivatives.velocity)
            .setEulerAngles(previous_state.getEulerAngles() + derivatives.eulerAngles)
            .setAngularVelocity(previous_state.getAngularVelocity() + derivatives.angular_velocity)
            .setTotalForce(interpolatedValues.total_force)
            .setTotalMoment(interpolatedValues.total_moment)
            .setInertia(interpolatedValues.inertia)
            .setMass(interpolatedValues.mass)
                .build();
    }

private:
    struct Derivatives {
        Eigen::Vector3<metricType> position;
        Eigen::Vector3<metricType> velocity;
        Eigen::Vector3<metricType> eulerAngles;
        Eigen::Vector3<metricType> angular_velocity;
    };
    struct InterpolatedValues {
        //ну сила и моменты не совсем интерполируемые, если брать методику полную, но я хз как еще назвать
        Eigen::Vector3<metricType> total_force;
        Eigen::Vector3<metricType> total_moment;
        Eigen::Vector3<metricType> inertia;
        metricType mass;
    };

    std::pair<Derivatives, InterpolatedValues> compute_complete_solution(
        const ObjSnapshot<metricType>& state, metricType t) {

        const auto& prev_position = state.getPosition();
        const auto& prev_velocity = state.getVelocity();
        const auto& prev_angularVel = state.getAngularVelocity();
        const auto& prev_eulerAngles = state.getEulerAngles();

        InterpolatedValues InterpolatedValues;
        InterpolatedValues.inertia = compute_inertia(t); // вот тут скорее всего состояние не потребуется
        InterpolatedValues.mass = compute_mass(t);
        InterpolatedValues.total_force = compute_total_force(state, t); // а вот тут потребуется, ибо ад силы рассчитываются из ад углов атаки и пространственного угла скольжения
        InterpolatedValues.total_moment = compute_total_moment(state, t); // как и тут

        Derivatives derivatives;
        derivatives.position = prev_velocity;// тут в земную систему координат перевести нужно(умножить на матрицу направляющих косинусов)
        derivatives.velocity = compute_velocity(prev_velocity,prev_angularVel, InterpolatedValues.total_force, InterpolatedValues.mass);
        derivatives.angular_velocity = compute_angular_velocity(InterpolatedValues.total_moment,InterpolatedValues.inertia,prev_angularVel);
        derivatives.eulerAngles = compute_angular_acceleration(prev_eulerAngles,prev_angularVel);

        return {derivatives, InterpolatedValues};
    }

    //вот тут подумать бы насчет того, как хранить и где хранить интерполируемые величины
    // и не забывай все
     Eigen::Vector3<metricType> compute_inertia(metricType t) {
        Eigen::Vector3<metricType> res = Eigen::Matrix<metricType, 3, 1>::Zero();
        return res;
    }
    metricType compute_mass(metricType t) {
        return 0.0;
    }

    Eigen::Vector3<metricType> compute_total_force(const ObjSnapshot<metricType> &state, metricType t) {
        return Eigen::Matrix<metricType, 3, 1>::Zero();
    }
    Eigen::Vector3<metricType> compute_total_moment(const ObjSnapshot<metricType>& state, metricType t) {
        return Eigen::Matrix<metricType, 3, 1>::Zero();
    }
    Eigen::Vector3<metricType>  compute_velocity( const Eigen::Vector3<metricType>& prev_velocity,
         Eigen::Vector3<metricType> prev_angularVel,
         Eigen::Vector3<metricType> total_force, const metricType mass ) {
        return Eigen::Matrix<metricType, 3, 1>::Zero();
    }
    Eigen::Vector3<metricType> compute_angular_velocity( const Eigen::Vector3<metricType>& total_moment,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& prev_angularVel ) {
        return Eigen::Matrix<metricType, 3, 1>::Zero();
    }
    Eigen::Vector3<metricType> compute_angular_acceleration( const Eigen::Vector3<metricType>& prev_eulerAngles,
        const Eigen::Vector3<metricType>& prev_angularVel ) {
        return Eigen::Matrix<metricType, 3, 1>::Zero();
    }

};


