#pragma once
#include "PCH.h"

template<typename metricType>
class KinematicState;
/**
 * @brief Тип для представления производной кинематического состояния.
 *
 * Этот класс представляет собой производную по времени от кинематического состояния (KinematicState),
 * то есть [dx/dt, dv/dt, deuler/dt, dAngVel/dt].
 * Он отделен от KinematicState для обеспечения типовой безопасности.
 */
template<typename metricType>
class KinematicStateDerivative {
public:
    KinematicStateDerivative(
        Eigen::Vector3<metricType> dPosition,
        Eigen::Vector3<metricType> dVelocity,
        Eigen::Vector3<metricType> dEulerAngles,
        Eigen::Vector3<metricType> dAngularVelocity
    ) : dPosition_(std::move(dPosition)),
        dVelocity_(std::move(dVelocity)),
        dEulerAngles_(std::move(dEulerAngles)),
        dAngularVelocity_(std::move(dAngularVelocity)) {}

    [[nodiscard]] const Eigen::Vector3<metricType>& getDPosition() const { return dPosition_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getDVelocity() const { return dVelocity_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getDEulerAngles() const { return dEulerAngles_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getDAngularVelocity() const { return dAngularVelocity_; }

    void setDPosition(const Eigen::Vector3<metricType>& pos) { dPosition_ = pos; }
    void setDVelocity(const Eigen::Vector3<metricType>& vel) { dVelocity_ = vel; }
    void setDEulerAngles(const Eigen::Vector3<metricType>& angles) { dEulerAngles_ = angles; }
    void setDAngularVelocity(const Eigen::Vector3<metricType>& angVel) { dAngularVelocity_ = angVel; }


    KinematicStateDerivative operator+(const KinematicStateDerivative& other) const {
        return KinematicStateDerivative(
            dPosition_ + other.dPosition_,
            dVelocity_ + other.dVelocity_,
            dEulerAngles_ + other.dEulerAngles_,
            dAngularVelocity_ + other.dAngularVelocity_
        );
    }

    KinematicStateDerivative operator*(metricType scalar) const {
        return KinematicStateDerivative(
            dPosition_ * scalar,
            dVelocity_ * scalar,
            dEulerAngles_ * scalar,
            dAngularVelocity_ * scalar
        );
    }

    friend KinematicStateDerivative operator*(metricType scalar, const KinematicStateDerivative& deriv) {
         return deriv * scalar;
    }

    friend KinematicState<metricType> operator+(const KinematicState<metricType>& state, const KinematicStateDerivative<metricType>& deriv_scaled) {
         // Предполагается, что deriv_scaled уже умножена на шаг времени
         return KinematicState<metricType>::createBuilder()
             .setPosition(state.getPosition() + deriv_scaled.dPosition_)
             .setVelocity(state.getVelocity() + deriv_scaled.dVelocity_)
             .setEulerAngles(state.getEulerAngles() + deriv_scaled.dEulerAngles_)
             .setAngularVelocity(state.getAngularVelocity() + deriv_scaled.dAngularVelocity_)
             .build();
    }

    friend KinematicState<metricType>& operator+=(KinematicState<metricType>& state, const KinematicStateDerivative<metricType>& deriv) {
         state.setPosition(state.getPosition() + deriv.dPosition_);
         state.setVelocity(state.getVelocity() + deriv.dVelocity_);
         state.setEulerAngles(state.getEulerAngles() + deriv.dEulerAngles_);
         state.setAngularVelocity(state.getAngularVelocity() + deriv.dAngularVelocity_);
         return state;
    }

private:
    Eigen::Vector3<metricType> dPosition_;
    Eigen::Vector3<metricType> dVelocity_;
    Eigen::Vector3<metricType> dEulerAngles_;
    Eigen::Vector3<metricType> dAngularVelocity_;
};

