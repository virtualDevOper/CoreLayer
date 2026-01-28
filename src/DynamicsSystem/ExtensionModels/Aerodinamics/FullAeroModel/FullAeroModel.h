#pragma once
#include "../AeroInput/RocketAeroInput.h"




template<typename metricType>
class FullAeroModel {
public:
    // ИЗМЕНЕННЫЙ КОНСТРУКТОР: принимаем ссылки вместо указателей
    explicit FullAeroModel(
        const RocketAeroInput<metricType>& aero_input,
        const std::shared_ptr<ComponentInterpolationManager<metricType>>& interp_mgr)
        : aero_input_(aero_input), interp_mgr_(interp_mgr) {

        if (!interp_mgr_) {
            throw std::invalid_argument("Interpolation manager cannot be null");
        }

        // Проверка корректности данных аэродинамики
        if (aero_input_.rudder_count <= 0 || aero_input_.D_mid <= 0) {
            throw std::invalid_argument("Invalid aerodynamic input parameters"); // ну это бред так-то
        }
    }

    // === УГЛЫ АТАКИ И ОРИЕНТАЦИИ ===
    metricType computeAlpha(const Eigen::Vector3<metricType>& velocity) const {
        metricType V_xy = std::sqrt(velocity(0)*velocity(0) + velocity(1)*velocity(1));
        if (V_xy < 1e-6) return 0;
        return -std::asin(velocity(1) / V_xy);
    }

    metricType computeBeta(const Eigen::Vector3<metricType>& velocity) const {
        metricType V_mag = velocity.norm();
        if (V_mag < 1e-6) return 0;
        return std::asin(velocity(2) / V_mag);
    }

    metricType computeAlphaP(const Eigen::Vector3<metricType>& velocity) const {
        metricType V_mag = velocity.norm();
        if (V_mag < 1e-6) return 0;
        metricType V_lateral = std::sqrt(velocity(1)*velocity(1) + velocity(2)*velocity(2));
        return std::asin(V_lateral / V_mag);
    }

    metricType computePhiP(const Eigen::Vector3<metricType>& velocity) const {
        metricType V_lateral = std::sqrt(velocity(1)*velocity(1) + velocity(2)*velocity(2));
        if (V_lateral < 1e-6) return 0;
        return -std::atan2(velocity(2), velocity(1));
    }

    // === ВЫЧИСЛЕНИЕ СИЛ ===
    Eigen::Vector3<metricType> computeAerodynamicForces(
        const Eigen::Vector3<metricType>& velocity,
        metricType rho) const {

        metricType V_mag = velocity.norm();
        if (V_mag < 1e-6) return Eigen::Vector3<metricType>::Zero();

        metricType alpha = computeAlpha(velocity);
        metricType mach = computeMach(V_mag);

        metricType q_dynamic = 0.5 * rho * V_mag * V_mag;
        metricType reference_area = M_PI * aero_input_.D_mid * aero_input_.D_mid / 4.0;

        // Получаем коэффициенты (в текущем ComponentInterpolationManager
        // коэффициенты зависят только от (alpha, mach))
        metricType C_x = interp_mgr_->getCxAero(alpha, mach);
        metricType C_y = interp_mgr_->getCyAero(alpha, mach);
        metricType C_z = interp_mgr_->getCzAero(alpha, mach);

        return q_dynamic * reference_area * Eigen::Vector3<metricType>(C_x, C_y, C_z);
    }

    // === ВЫЧИСЛЕНИЕ МОМЕНТОВ ===
    Eigen::Vector3<metricType> computeAerodynamicMoments(
        const Eigen::Vector3<metricType>& velocity,
        const Eigen::Vector3<metricType>& angular_velocity,
        metricType rho,
        metricType center_of_mass,
        const std::vector<metricType>& rudder_deflections) const {

        if (rudder_deflections.size() < static_cast<size_t>(aero_input_.rudder_count)) {
            throw std::invalid_argument("Insufficient rudder deflections provided");
        }

        metricType V_mag = velocity.norm();
        if (V_mag < 1e-6) return Eigen::Vector3<metricType>::Zero();

        // Вычисляем углы атаки
        metricType alpha = computeAlpha(velocity);
        metricType beta = computeBeta(velocity);
        metricType alpha_p = computeAlphaP(velocity);
        metricType phi_p = computePhiP(velocity);
        metricType mach = computeMach(V_mag);

        // Динамическое давление
        metricType q_dynamic = 0.5 * rho * V_mag * V_mag;
        metricType reference_area = M_PI * aero_input_.D_mid * aero_input_.D_mid / 4.0;

        // === МОМЕНТ КРЕНА (X) ===
        metricType moment_x = computeMomentX(
            alpha, beta, alpha_p, phi_p, mach,
            angular_velocity(0), V_mag, rudder_deflections,
            q_dynamic, reference_area);

        // === МОМЕНТ ТАНГАЖА (Y) ===
        metricType moment_y = computeMomentY(
            alpha, beta, alpha_p, phi_p, mach,
            angular_velocity(1), V_mag, center_of_mass, rudder_deflections,
            q_dynamic, reference_area);

        // === МОМЕНТ РЫСКАНИЯ (Z) ===
        metricType moment_z = computeMomentZ(
            alpha, beta, alpha_p, phi_p, mach,
            angular_velocity(2), V_mag, center_of_mass, rudder_deflections,
            q_dynamic, reference_area);

        return Eigen::Vector3<metricType>(moment_x, moment_y, moment_z);
    }

    // === ОБНОВЛЕНИЕ УГЛОВ ОТКЛОНЕНИЯ РУЛЕЙ ===
    void updateControlSurfaceDeflections(const std::vector<metricType>& deflections) {
        if (deflections.size() != static_cast<size_t>(aero_input_.rudder_count)) {
            throw std::invalid_argument("Deflection count mismatch");
        }
        current_deflections_ = deflections;
    }

private:
    const RocketAeroInput<metricType>& aero_input_;
    std::shared_ptr<ComponentInterpolationManager<metricType>> interp_mgr_;
    std::vector<metricType> current_deflections_;

    // === ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===
    metricType computeMach(metricType velocity) const {
        // a = 20.04 * sqrt(288.16) ≈ 340.3 m/s
        metricType speed_of_sound = 20.04 * std::sqrt(288.16);
        return velocity / speed_of_sound;
    }

    // === МОМЕНТ КРЕНА ===
    metricType computeMomentX(
        metricType /*alpha*/, metricType /*beta*/, metricType /*alpha_p*/, metricType /*phi_p*/,
        metricType /*mach*/, metricType /*wx*/, metricType /*V_mag*/,
        const std::vector<metricType>& /*rudder_deflections*/,
        metricType /*q_dynamic*/, metricType /*reference_area*/) const {

        // TODO: детализированная модель моментов по крену требует
        // дополнительных аэродинамических производных, которых пока нет
        // в ComponentInterpolationManager. Временная заглушка.
        return static_cast<metricType>(0);
    }

    // === МОМЕНТ ТАНГАЖА ===
    metricType computeMomentY(
        metricType /*alpha*/, metricType /*beta*/, metricType /*alpha_p*/, metricType /*phi_p*/,
        metricType /*mach*/, metricType /*wy*/, metricType /*V_mag*/, metricType /*Xm*/,
        const std::vector<metricType>& /*rudder_deflections*/,
        metricType /*q_dynamic*/, metricType /*reference_area*/) const {

        // TODO: аналогично computeMomentX, полноценная модель требует
        // набора аэродинамических производных, которые пока не реализованы.
        return static_cast<metricType>(0);
    }

    // === МОМЕНТ РЫСКАНИЯ ===
    metricType computeMomentZ(
        metricType /*alpha*/, metricType /*beta*/, metricType /*alpha_p*/, metricType /*phi_p*/,
        metricType /*mach*/, metricType /*wz*/, metricType /*V_mag*/, metricType /*Xm*/,
        const std::vector<metricType>& /*rudder_deflections*/,
        metricType /*q_dynamic*/, metricType /*reference_area*/) const {

        // TODO: заглушка для рыскательного момента по тем же причинам,
        // что и в computeMomentX/computeMomentY.
        return static_cast<metricType>(0);
    }

    // === ВСПОМОГАТЕЛЬНЫЕ ВЫЧИСЛЕНИЯ ===
    metricType computeAlphaSt(metricType alpha_p, metricType phi_p, int i) const {
        metricType phi_st_i = getPhi0St(i) + phi_p;
        metricType tan_a_st = std::tan(alpha_p) * std::cos(phi_st_i)
                            + 0.199 * std::tan(alpha_p) * std::sin(alpha_p) * std::sin(2.0 * phi_st_i);
        return std::atan(tan_a_st);
    }

    metricType computeAlphaR(metricType alpha_p, metricType phi_p, int i,
                            const std::vector<metricType>& rudder_deflections) const {
        metricType phi_r_i = getPhi0R(i) + phi_p;
        metricType tan_a_r = 1.361 * std::tan(alpha_p) * std::cos(phi_r_i)
                           + 0.25 * std::tan(alpha_p) * std::sin(alpha_p) * std::sin(2.0 * phi_r_i);

        metricType lambda_sum = 0;
        for (int j = 0; j < aero_input_.rudder_count; j++) {
            lambda_sum += getLambdaBig(i, j) * rudder_deflections[j];
        }
        
        return std::atan(tan_a_r) + lambda_sum;
    }

    metricType getLambdaBig(int i, int j) const {
        if (i == j % 4) return 0.8892;
        else if (j == (i + 1) % 4) return -0.1102;
        else if (j == (i + 2) % 4) return 0.0497;
        else return -0.1102;
    }

    metricType getPhi0St(int i) const {
        const metricType phi_0_st[] = {0, M_PI/2, M_PI, 3*M_PI/2};
        return phi_0_st[i % 4];
    }

    metricType getPhi0R(int i) const {
        const metricType phi_0_r[] = {45*M_PI/180, 135*M_PI/180, 225*M_PI/180, 315*M_PI/180};
        return phi_0_r[i % 4];
    }
};