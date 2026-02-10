#pragma once
#include "PCH.h"

/**
 * \brief Иммутабельное кинематическое состояние объекта для численного интегрирования.
 * 
 * \tparam metricType Тип данных для метрических величин.
 * 
 * \details Содержит только переменные, участвующие в методе Рунге-Кутта 4-го порядка:
 * позицию, скорость, углы Эйлера и угловую скорость. Физические параметры
 * (масса, инерция, силы) не входят - они интерполируются отдельно.
 * Поддерживает арифметические операции для численного интегрирования.
 * Создается только через Builder для обеспечения корректности данных.
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
        Builder& setPosition(const Eigen::Vector3<metricType>& v);
        Builder& setVelocity(const Eigen::Vector3<metricType>& v);
        Builder& setEulerAngles(const Eigen::Vector3<metricType>& v);
        Builder& setAngularVelocity(const Eigen::Vector3<metricType>& v);
        [[nodiscard]] KinematicState build() const;
    };
    static Builder createBuilder();

    // === ГЕТТЕРЫ (только константные) ===
    [[nodiscard]] const Eigen::Vector3<metricType>& getPosition() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getVelocity() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getEulerAngles() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getAngularVelocity() const;

    // === ENU-SPECIFIC ACCESSORS ===
    // Position components in ENU coordinates
    [[nodiscard]] metricType getEast() const { return position_.x(); }
    [[nodiscard]] metricType getNorth() const { return position_.y(); }
    [[nodiscard]] metricType getAltitude() const { return position_.z(); }
    
    // Velocity components in ENU coordinates
    [[nodiscard]] metricType getVelocityEast() const { return velocity_.x(); }
    [[nodiscard]] metricType getVelocityNorth() const { return velocity_.y(); }
    [[nodiscard]] metricType getVelocityUp() const { return velocity_.z(); }

    // === АРИФМЕТИЧЕСКИЕ ОПЕРАТОРЫ ДЛЯ РК4 ===
    KinematicState operator+(const KinematicState& other) const;
    KinematicState operator*(metricType scalar) const;

private:
    // Приватный конструктор — только через Builder
    KinematicState(
        Eigen::Vector3<metricType> position,
        Eigen::Vector3<metricType> velocity,
        Eigen::Vector3<metricType> eulerAngles,
        Eigen::Vector3<metricType> angularVelocity
    );

    Eigen::Vector3<metricType> position_;
    Eigen::Vector3<metricType> velocity_;
    Eigen::Vector3<metricType> eulerAngles_;
    Eigen::Vector3<metricType> angularVelocity_;
};

// Глобальный оператор скаляр * состояние (для симметрии)
template<typename metricType>
KinematicState<metricType> operator*(metricType scalar, const KinematicState<metricType>& state);