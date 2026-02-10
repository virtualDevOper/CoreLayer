#pragma once
#include "PCH.h"
#include "KinematicState/KinematicState.h"  // ← НОВАЯ ЗАВИСИМОСТЬ

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
    ObjSnapshot();

    // === ДЕЛЕГИРУЮЩИЕ ГЕТТЕРЫ ДЛЯ ОБРАТНОЙ СОВМЕСТИМОСТИ ===
    [[nodiscard]] const Eigen::Vector3<metricType>& getPosition() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getVelocity() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getEulerAngles() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getAngularVelocity() const;

    // === ГЕТТЕРЫ ПАРАМЕТРОВ ===
    [[nodiscard]] metricType getTime() const;
    [[nodiscard]] metricType getMass() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getInertia() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalForce() const;
    [[nodiscard]] const Eigen::Vector3<metricType>& getTotalMoment() const;

    // === ДОСТУП К КИНЕМАТИКЕ ===
    [[nodiscard]] const KinematicState<metricType>& getKinematics() const;

    // === BUILDER С ИММУТАБЕЛЬНОСТЬЮ ===
    class Builder {
    private:
        KinematicState<metricType> kinematics_;
        metricType time_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType mass_ = std::numeric_limits<metricType>::quiet_NaN();
        Eigen::Vector3<metricType> inertia_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalForce_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());
        Eigen::Vector3<metricType> totalMoment_ = Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN());

    public:
        explicit Builder(const KinematicState<metricType>& kinematics) : kinematics_(kinematics) {}
        static Builder fromKinematics(const KinematicState<metricType>& kinematics) {
            return Builder(kinematics);
        }
        Builder& setTime(metricType t);
        Builder& setMass(metricType m);
        Builder& setInertia(const Eigen::Vector3<metricType>& i);
        Builder& setTotalForce(const Eigen::Vector3<metricType>& f);
        Builder& setTotalMoment(const Eigen::Vector3<metricType>& m);
        [[nodiscard]] std::unique_ptr<ObjSnapshot<metricType>> buildUnique() const;
    };
    static Builder createBuilder(const KinematicState<metricType>& kinematics);

    // === ПАРАМЕТРЫ ДЛЯ СОХРАНЕНИЯ ===
    [[nodiscard]] std::map<std::string, metricType> getParams() const;

private:
    KinematicState<metricType> kinematics_;
    metricType time_;
    metricType mass_;
    Eigen::Vector3<metricType> inertia_;
    Eigen::Vector3<metricType> totalForce_;
    Eigen::Vector3<metricType> totalMoment_;
};