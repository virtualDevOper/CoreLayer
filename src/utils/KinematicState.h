#pragma once
#include "../../PCH.h"

/**
 * \brief Иммутабельное состояние кинематических переменных для численного интегрирования
 * 
 * Содержит ТОЛЬКО переменные, участвующие в РК4:
 * - Позиция (3 компоненты)
 * - Скорость (3 компоненты)
 * - Углы Эйлера (3 компоненты)
 * - Угловая скорость (3 компоненты)
 * 
 * Параметры (масса, инерция, силы) НЕ входят — они интерполируются напрямую через augmentSnapshot()
 */
template<typename metricType>
class KinematicState {
public:
    // === ИММУТАБЕЛЬНЫЙ КОНСТРУКТОР ЧЕРЕЗ BUILDER ===
    class Builder {
    private:
        Eigen::Vector3<metricType> position_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> velocity_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> eulerAngles_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> angularVelocity_ = Eigen::Vector3<metricType>::Zero();
    public:
        Builder& setPosition(const Eigen::Vector3<metricType>& v) { position_ = v; return *this; }
        Builder& setVelocity(const Eigen::Vector3<metricType>& v) { velocity_ = v; return *this; }
        Builder& setEulerAngles(const Eigen::Vector3<metricType>& v) { eulerAngles_ = v; return *this; }
        Builder& setAngularVelocity(const Eigen::Vector3<metricType>& v) { angularVelocity_ = v; return *this; }
        [[nodiscard]] KinematicState build() const {
            return KinematicState(position_, velocity_, eulerAngles_, angularVelocity_);
        }
    };
    static Builder createBuilder() { return Builder(); }

    // === ГЕТТЕРЫ (только константные) ===
    [[nodiscard]] const Eigen::Vector3<metricType>& getPosition() const { return position_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getVelocity() const { return velocity_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getEulerAngles() const { return eulerAngles_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getAngularVelocity() const { return angularVelocity_; }

    // === АРИФМЕТИЧЕСКИЕ ОПЕРАТОРЫ ДЛЯ РК4 ===
    KinematicState operator+(const KinematicState& other) const {
        return KinematicState(
            position_ + other.position_,
            velocity_ + other.velocity_,
            eulerAngles_ + other.eulerAngles_,
            angularVelocity_ + other.angularVelocity_
        );
    }
    KinematicState operator*(metricType scalar) const {
        return KinematicState(
            position_ * scalar,
            velocity_ * scalar,
            eulerAngles_ * scalar,
            angularVelocity_ * scalar
        );
    }

private:
    // Приватный конструктор — только через Builder
    KinematicState(
        Eigen::Vector3<metricType> position,
        Eigen::Vector3<metricType> velocity,
        Eigen::Vector3<metricType> eulerAngles,
        Eigen::Vector3<metricType> angularVelocity
    ) : position_(std::move(position)),
        velocity_(std::move(velocity)),
        eulerAngles_(std::move(eulerAngles)),
        angularVelocity_(std::move(angularVelocity)) {}

    Eigen::Vector3<metricType> position_;
    Eigen::Vector3<metricType> velocity_;
    Eigen::Vector3<metricType> eulerAngles_;
    Eigen::Vector3<metricType> angularVelocity_;
};

// Глобальный оператор скаляр * состояние (для симметрии)
template<typename metricType>
inline KinematicState<metricType> operator*(metricType scalar, const KinematicState<metricType>& state) {
    return state * scalar;
}