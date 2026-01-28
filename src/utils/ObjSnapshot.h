#pragma once
#include "../../PCH.h"
#include "KinematicState.h"  // ← НОВАЯ ЗАВИСИМОСТЬ

/**
 * \brief Полный снимок состояния объекта (кинематика + параметры)
 *
 * Архитектура:
 * - kinematics_ — интегрируемые переменные (участвуют в РК4)
 * - параметры (масса, инерция...) — интерполируются напрямую, НЕ интегрируются
 *
 * Иммутабельность сохранена: все изменения через Builder
 */
template<typename metricType>
class ObjSnapshot {
public:
    ObjSnapshot()
        : kinematics_(KinematicState<metricType>::createBuilder().build()),
          time_(std::numeric_limits<metricType>::quiet_NaN()),
          mass_(std::numeric_limits<metricType>::quiet_NaN()),
          inertia_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
          totalForce_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
          totalMoment_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
          totalForceEarth_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
          totalMomentEarth_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())) {}

    // === ДЕЛЕГИРУЮЩИЕ ГЕТТЕРЫ ДЛЯ ОБРАТНОЙ СОВМЕСТИМОСТИ ===
    [[nodiscard]] const Eigen::Vector3<metricType>& getPosition() const { return kinematics_.getPosition(); }
    [[nodiscard]] const Eigen::Vector3<metricType>& getVelocity() const { return kinematics_.getVelocity(); }
    [[nodiscard]] const Eigen::Vector3<metricType>& getEulerAngles() const { return kinematics_.getEulerAngles(); }
    [[nodiscard]] const Eigen::Vector3<metricType>& getAngularVelocity() const { return kinematics_.getAngularVelocity(); }

    // === ГЕТТЕРЫ ПАРАМЕТРОВ ===
    [[nodiscard]] metricType getTime() const { return time_; }
    [[nodiscard]] metricType getMass() const { return mass_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getInertia() const { return inertia_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalForce() const { return totalForce_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalMoment() const { return totalMoment_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalForceEarth() const { return totalForceEarth_; }
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalMomentEarth() const { return totalMomentEarth_; }

    // === ДОСТУП К КИНЕМАТИКЕ ===
    [[nodiscard]] const KinematicState<metricType>& getKinematics() const { return kinematics_; }

    // === BUILDER С ИММУТАБЕЛЬНОСТЬЮ ===
    class Builder {
    private:
        KinematicState<metricType> kinematics_;
        metricType time_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType mass_ = std::numeric_limits<metricType>::quiet_NaN();
        Eigen::Vector3<metricType> inertia_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalForce_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalMoment_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalForceEarth_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalMomentEarth_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
    public:
        explicit Builder(const KinematicState<metricType>& kinematics) : kinematics_(kinematics) {}
        static Builder fromKinematics(const KinematicState<metricType>& kinematics) {
            return Builder(kinematics);
        }
        Builder& setTime(metricType t) { time_ = t; return *this; }
        Builder& setMass(metricType m) { mass_ = m; return *this; }
        Builder& setInertia(const Eigen::Vector3<metricType>& i) { inertia_ = i; return *this; }
        Builder& setTotalForce(const Eigen::Vector3<metricType>& f) { totalForce_ = f; return *this; }
        Builder& setTotalMoment(const Eigen::Vector3<metricType>& m) { totalMoment_ = m; return *this; }
        Builder& setTotalForceEarth(const Eigen::Vector3<metricType>& f) { totalForceEarth_ = f; return *this; }
        Builder& setTotalMomentEarth(const Eigen::Vector3<metricType>& m) { totalMomentEarth_ = m; return *this; }
        [[nodiscard]] std::unique_ptr<ObjSnapshot<metricType>> buildUnique() const {
            auto snapshot = std::make_unique<ObjSnapshot<metricType>>();
            snapshot->kinematics_ = kinematics_;
            snapshot->time_ = time_;
            snapshot->mass_ = mass_;
            snapshot->inertia_ = inertia_;
            snapshot->totalForce_ = totalForce_;
            snapshot->totalMoment_ = totalMoment_;
            snapshot->totalForceEarth_ = totalForceEarth_;
            snapshot->totalMomentEarth_ = totalMomentEarth_;
            return snapshot;
        }
    };
    static Builder createBuilder(const KinematicState<metricType>& kinematics) {
        return Builder::fromKinematics(kinematics);
    }

    // === ПАРАМЕТРЫ ДЛЯ СОХРАНЕНИЯ ===
    [[nodiscard]] std::map<std::string, metricType> getParams() const {
        std::map<std::string, metricType> params;
        // Время (если известно)
        if (!std::isnan(time_)) params["time"] = time_;

        // Кинематика
        const auto& pos = kinematics_.getPosition();
        const auto& vel = kinematics_.getVelocity();
        const auto& euler = kinematics_.getEulerAngles();
        const auto& ang_vel = kinematics_.getAngularVelocity();
        params["position_x"] = pos.x(); params["position_y"] = pos.y(); params["position_z"] = pos.z();
        params["velocity_x"] = vel.x(); params["velocity_y"] = vel.y(); params["velocity_z"] = vel.z();
        params["eulerAngles_x"] = euler.x(); params["eulerAngles_y"] = euler.y(); params["eulerAngles_z"] = euler.z();
        params["angularVelocity_x"] = ang_vel.x(); params["angularVelocity_y"] = ang_vel.y(); params["angularVelocity_z"] = ang_vel.z();
        // Параметры
        if (!std::isnan(mass_)) params["mass"] = mass_;
        if (!std::isnan(inertia_.x())) params["inertia_x"] = inertia_.x();
        if (!std::isnan(inertia_.y())) params["inertia_y"] = inertia_.y();
        if (!std::isnan(inertia_.z())) params["inertia_z"] = inertia_.z();
        if (!std::isnan(totalForce_.x())) params["totalForce_x"] = totalForce_.x();
        if (!std::isnan(totalForce_.y())) params["totalForce_y"] = totalForce_.y();
        if (!std::isnan(totalForce_.z())) params["totalForce_z"] = totalForce_.z();
        if (!std::isnan(totalMoment_.x())) params["totalMoment_x"] = totalMoment_.x();
        if (!std::isnan(totalMoment_.y())) params["totalMoment_y"] = totalMoment_.y();
        if (!std::isnan(totalMoment_.z())) params["totalMoment_z"] = totalMoment_.z();
        // Силы и моменты в земной СК (если заданы)
        if (!std::isnan(totalForceEarth_.x())) params["totalForceEarth_x"] = totalForceEarth_.x();
        if (!std::isnan(totalForceEarth_.y())) params["totalForceEarth_y"] = totalForceEarth_.y();
        if (!std::isnan(totalForceEarth_.z())) params["totalForceEarth_z"] = totalForceEarth_.z();
        if (!std::isnan(totalMomentEarth_.x())) params["totalMomentEarth_x"] = totalMomentEarth_.x();
        if (!std::isnan(totalMomentEarth_.y())) params["totalMomentEarth_y"] = totalMomentEarth_.y();
        if (!std::isnan(totalMomentEarth_.z())) params["totalMomentEarth_z"] = totalMomentEarth_.z();
        return params;
    }

private:
    KinematicState<metricType> kinematics_;  // ← ЕДИНСТВЕННОЕ ИЗМЕНЕНИЕ СТРУКТУРЫ
    metricType time_;
    metricType mass_;
    Eigen::Vector3<metricType> inertia_;
    Eigen::Vector3<metricType> totalForce_;
    Eigen::Vector3<metricType> totalMoment_;
    Eigen::Vector3<metricType> totalForceEarth_;
    Eigen::Vector3<metricType> totalMomentEarth_;
};