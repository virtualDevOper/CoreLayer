//
// Created by 4NR_Operator_3 on 24.12.2025.
//

#pragma once
#include "PCH.h"
#include "../ParameterProvider/IParameterProvider.h"
#include "ILinearInterpolator/ILinearInterpolator.h"
#include "IBilinearInterpolator/IBilinearInterpolator.h"

/**
 * \brief Гибкий менеджер интерполяции компонентов с опциональными полями
 *
 * \details ВСЕ интерполяторы опциональны. Ла создаёт
 * те, что нужны. Аккуратнее с исключениями.
 * !!!!! По мере наполнения программы могут появляться новые интерполируемые величины
 * так что можешь просто добавлять их сюда, всего учесть сразу не могу, вдруг ты
 * захочешь скорость интерполировать)))
 */
template<typename metricType>
class ComponentInterpolationManager final
    : public IParameterProvider<metricType>
{
private:
    // === МАССА И ИНЕРЦИЯ ===
    std::unique_ptr<ILinearInterpolator<metricType>> mass_;
    std::unique_ptr<ILinearInterpolator<metricType>> inertia_x_;
    std::unique_ptr<ILinearInterpolator<metricType>> inertia_y_;
    std::unique_ptr<ILinearInterpolator<metricType>> inertia_z_;

    // === ЦЕНТР МАСС ===
    std::unique_ptr<ILinearInterpolator<metricType>> COM_x_;
    std::unique_ptr<ILinearInterpolator<metricType>> COM_y_;
    std::unique_ptr<ILinearInterpolator<metricType>> COM_z_;

    // === ТЯГА ===
    std::unique_ptr<ILinearInterpolator<metricType>> thrust_x_;
    std::unique_ptr<ILinearInterpolator<metricType>> thrust_y_;
    std::unique_ptr<ILinearInterpolator<metricType>> thrust_z_;

    // === АЭРОДИНАМИЧЕСКИЕ КОЭФФИЦИЕНТЫ СИЛ ===
    std::unique_ptr<IBilinearInterpolator<metricType>> cx_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> cy_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> cz_aero_;

    // === АЭРОДИНАМИЧЕСКИЕ ПРОИЗВОДНЫЕ ===
    std::unique_ptr<IBilinearInterpolator<metricType>> cy_r_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> cy_k_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> cy_st_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> xd_k_aero_;
    std::unique_ptr<IBilinearInterpolator<metricType>> a_ck_otn_aero_;

public:
    ComponentInterpolationManager() = default;
    ~ComponentInterpolationManager() override = default;

    // ========================================================================
    // СЕТТЕРЫ
    // ========================================================================

    void setMass(std::unique_ptr<ILinearInterpolator<metricType>> interp) {
        mass_ = std::move(interp);
    }

    void setCOM(
        std::unique_ptr<ILinearInterpolator<metricType>> x,
        std::unique_ptr<ILinearInterpolator<metricType>> y,
        std::unique_ptr<ILinearInterpolator<metricType>> z) {
        COM_x_ = std::move(x);
        COM_y_ = std::move(y);
        COM_z_ = std::move(z);
    }

    void setInertia(
        std::unique_ptr<ILinearInterpolator<metricType>> ix,
        std::unique_ptr<ILinearInterpolator<metricType>> iy,
        std::unique_ptr<ILinearInterpolator<metricType>> iz) {
        inertia_x_ = std::move(ix);
        inertia_y_ = std::move(iy);
        inertia_z_ = std::move(iz);
    }

    void setThrust(
        std::unique_ptr<ILinearInterpolator<metricType>> fx,
        std::unique_ptr<ILinearInterpolator<metricType>> fy,
        std::unique_ptr<ILinearInterpolator<metricType>> fz) {
        thrust_x_ = std::move(fx);
        thrust_y_ = std::move(fy);
        thrust_z_ = std::move(fz);
    }

    void setAerodynamicForceCoefficients(
        std::unique_ptr<IBilinearInterpolator<metricType>> cx,
        std::unique_ptr<IBilinearInterpolator<metricType>> cy,
        std::unique_ptr<IBilinearInterpolator<metricType>> cz) {
        cx_aero_ = std::move(cx);
        cy_aero_ = std::move(cy);
        cz_aero_ = std::move(cz);
    }

    void setAerodynamicDerivatives(
        std::unique_ptr<IBilinearInterpolator<metricType>> cy_r,
        std::unique_ptr<IBilinearInterpolator<metricType>> cy_k,
        std::unique_ptr<IBilinearInterpolator<metricType>> cy_st,
        std::unique_ptr<IBilinearInterpolator<metricType>> xd_k,
        std::unique_ptr<IBilinearInterpolator<metricType>> a_ck_otn) {
        cy_r_aero_ = std::move(cy_r);
        cy_k_aero_ = std::move(cy_k);
        cy_st_aero_ = std::move(cy_st);
        xd_k_aero_ = std::move(xd_k);
        a_ck_otn_aero_ = std::move(a_ck_otn);
    }

    // ========================================================================
    // РЕАЛИЗАЦИЯ СТАРОГО КОНТРАКТА (для обратной совместимости)
    // ========================================================================

    [[nodiscard]] metricType getMass(metricType t) const override {
        if (!mass_) {
            throw std::runtime_error("Интерполятор массы не установлен");
        }
        return mass_->interpolate(t);
    }

    [[nodiscard]] Eigen::Vector3<metricType> getInertia(metricType t) const override {
        if (!inertia_x_ || !inertia_y_ || !inertia_z_) {
            throw std::runtime_error("Интерполяторы моментов инерции не установлены");
        }
        return Eigen::Vector3<metricType>(
            inertia_x_->interpolate(t),
            inertia_y_->interpolate(t),
            inertia_z_->interpolate(t)
        );
    }

    [[nodiscard]] Eigen::Vector3<metricType> getThrust(metricType t) const override {
        if (!thrust_x_ || !thrust_y_ || !thrust_z_) {
            throw std::runtime_error("Интерполяторы тяги не установлены");
        }
        return Eigen::Vector3<metricType>(
            thrust_x_->interpolate(t),
            thrust_y_->interpolate(t),
            thrust_z_->interpolate(t)
        );
    }

    [[nodiscard]] Eigen::Vector3<metricType> getCOM(metricType t) const override {
        if (!COM_x_ || !COM_y_ || !COM_z_) {
            throw std::runtime_error("Интерполяторы COM не установлены");
        }
        return Eigen::Vector3<metricType>(
            COM_x_->interpolate(t),
            COM_y_->interpolate(t),
            COM_z_->interpolate(t)
        );
    }

    // ========================================================================
    // РЕАЛИЗАЦИЯ НОВОГО КОНТРАКТА (std::optional, без исключений)
    // ========================================================================

    [[nodiscard]] std::optional<metricType> tryGetMass(metricType t) const override {
        if (!mass_) {
            return std::nullopt;
        }
        return mass_->interpolate(t);
    }

    [[nodiscard]] bool hasMass() const override {
        return mass_ != nullptr;
    }

    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetInertia(metricType t) const override {
        if (!inertia_x_ || !inertia_y_ || !inertia_z_) {
            return std::nullopt;
        }
        return Eigen::Vector3<metricType>(
            inertia_x_->interpolate(t),
            inertia_y_->interpolate(t),
            inertia_z_->interpolate(t)
        );
    }

    [[nodiscard]] bool hasInertia() const override {
        return inertia_x_ && inertia_y_ && inertia_z_;
    }

    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetThrust(metricType t) const override {
        if (!thrust_x_ || !thrust_y_ || !thrust_z_) {
            return std::nullopt;
        }
        return Eigen::Vector3<metricType>(
            thrust_x_->interpolate(t),
            thrust_y_->interpolate(t),
            thrust_z_->interpolate(t)
        );
    }

    [[nodiscard]] bool hasThrust() const override {
        return thrust_x_ && thrust_y_ && thrust_z_;
    }

    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetCOM(metricType t) const override {
        if (!COM_x_ || !COM_y_ || !COM_z_) {
            return std::nullopt;
        }
        return Eigen::Vector3<metricType>(
            COM_x_->interpolate(t),
            COM_y_->interpolate(t),
            COM_z_->interpolate(t)
        );
    }

    [[nodiscard]] bool hasCOM() const override {
        return COM_x_ && COM_y_ && COM_z_;
    }

    // ========================================================================
    // АЭРОДИНАМИЧЕСКИЕ МЕТОДЫ (для внутреннего использования)
    // ========================================================================

    [[nodiscard]] metricType getCxAero(metricType alpha, metricType mach) const {
        if (!cx_aero_) {
            throw std::runtime_error("Интерполятор Cx не установлен");
        }
        return cx_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getCyAero(metricType alpha, metricType mach) const {
        if (!cy_aero_) {
            throw std::runtime_error("Интерполятор Cy не установлен");
        }
        return cy_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getCzAero(metricType alpha, metricType mach) const {
        if (!cz_aero_) {
            throw std::runtime_error("Интерполятор Cz не установлен");
        }
        return cz_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] Eigen::Vector3<metricType> getAerodynamicForceCoefficients(
        metricType alpha, metricType mach) const {
        return Eigen::Vector3<metricType>(
            getCxAero(alpha, mach),
            getCyAero(alpha, mach),
            getCzAero(alpha, mach)
        );
    }

    [[nodiscard]] metricType getCyRAero(metricType alpha, metricType mach) const {
        if (!cy_r_aero_) {
            throw std::runtime_error("Интерполятор Cy_r не установлен");
        }
        return cy_r_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getCyKAero(metricType alpha, metricType mach) const {
        if (!cy_k_aero_) {
            throw std::runtime_error("Интерполятор Cy_k не установлен");
        }
        return cy_k_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getCyStAero(metricType alpha, metricType mach) const {
        if (!cy_st_aero_) {
            throw std::runtime_error("Интерполятор Cy_st не установлен");
        }
        return cy_st_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getXdKAero(metricType alpha, metricType mach) const {
        if (!xd_k_aero_) {
            throw std::runtime_error("Интерполятор Xd_k не установлен");
        }
        return xd_k_aero_->interpolate(alpha, mach);
    }

    [[nodiscard]] metricType getAckOtnAero(metricType alpha, metricType mach) const {
        if (!a_ck_otn_aero_) {
            throw std::runtime_error("Интерполятор a_ck_otn не установлен");
        }
        return a_ck_otn_aero_->interpolate(alpha, mach);
    }
};