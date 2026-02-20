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
    
    // === ГЕТТЕРЫ АЭРОДИНАМИЧЕСКИХ КОЭФФИЦИЕНТОВ ===
    [[nodiscard]] metricType getAeroCx() const;
    [[nodiscard]] metricType getAeroCy() const;
    [[nodiscard]] metricType getAeroCz() const;
    [[nodiscard]] metricType getAeroMx() const;
    [[nodiscard]] metricType getAeroMy() const;
    [[nodiscard]] metricType getAeroMz() const;
    [[nodiscard]] metricType getAeroAlpha() const;
    [[nodiscard]] metricType getAeroBeta() const;
    [[nodiscard]] metricType getAeroMach() const;
    [[nodiscard]] metricType getAeroXcp() const;
    [[nodiscard]] metricType getAeroStaticMargin() const;

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
        
        // Аэродинамические коэффициенты
        metricType aero_Cx_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_Cy_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_Cz_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_mx_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_my_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_mz_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_alpha_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_beta_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_mach_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_x_cp_ = std::numeric_limits<metricType>::quiet_NaN();
        metricType aero_static_margin_ = std::numeric_limits<metricType>::quiet_NaN();

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
        
        // Методы для установки аэродинамических коэффициентов
        Builder& setAeroCx(metricType cx);
        Builder& setAeroCy(metricType cy);
        Builder& setAeroCz(metricType cz);
        Builder& setAeroMx(metricType mx);
        Builder& setAeroMy(metricType my);
        Builder& setAeroMz(metricType mz);
        Builder& setAeroAlpha(metricType alpha);
        Builder& setAeroBeta(metricType beta);
        Builder& setAeroMach(metricType mach);
        Builder& setAeroXcp(metricType x_cp);
        Builder& setAeroStaticMargin(metricType static_margin);
        
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
    
    // Аэродинамические коэффициенты
    metricType aero_Cx_;
    metricType aero_Cy_;
    metricType aero_Cz_;
    metricType aero_mx_;
    metricType aero_my_;
    metricType aero_mz_;
    metricType aero_alpha_;
    metricType aero_beta_;
    metricType aero_mach_;
    metricType aero_x_cp_;
    metricType aero_static_margin_;
};