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
        metricType beta = computeBeta(velocity);
        metricType mach = computeMach(V_mag);

        metricType q_dynamic = 0.5 * rho * V_mag * V_mag;
        metricType reference_area = M_PI * aero_input_.D_mid * aero_input_.D_mid / 4.0;

        // Получаем коэффициенты
        metricType C_x = interp_mgr_->getCxAero(alpha, mach);
        metricType C_y = interp_mgr_->getCyAero(alpha, beta, mach);
        metricType C_z = interp_mgr_->getCzAero(alpha, beta, mach);

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
        metricType alpha, metricType beta, metricType alpha_p, metricType phi_p,
        metricType mach, metricType wx, metricType V_mag,
        const std::vector<metricType>& rudder_deflections,
        metricType q_dynamic, metricType reference_area) const {

        metricType wx_bezr = wx * 0.251 / V_mag;

        // Компоненты от стабилизаторов и рулей
        metricType first_component = 0, second_component = 0;
        for (int i = 0; i < aero_input_.rudder_count; i++) {
            metricType alpha_st_i = computeAlphaSt(alpha_p, phi_p, i);
            metricType alpha_r_i = computeAlphaR(alpha_p, phi_p, i, rudder_deflections);

            first_component += interp_mgr_->getCyKonsStAero(mach, alpha_st_i);
            second_component += interp_mgr_->getCyKonspAero(mach, alpha_r_i, rudder_deflections[i]);
        }

        metricType mx_static = -first_component * ((0.0355 + 0.09 * 0.45) / 0.251)
                              - second_component * ((0.0355 + 0.05 * 0.45) / 0.251);

        metricType mx_damping = interp_mgr_->getMxWxAero(mach) * wx_bezr;

        return q_dynamic * reference_area * aero_input_.L_sum_kr * (mx_static + mx_damping);
    }

    // === МОМЕНТ ТАНГАЖА ===
    metricType computeMomentY(
        metricType alpha, metricType beta, metricType alpha_p, metricType phi_p,
        metricType mach, metricType wy, metricType V_mag, metricType Xm,
        const std::vector<metricType>& rudder_deflections,
        metricType q_dynamic, metricType reference_area) const {

        metricType wy_bezr = wy * aero_input_.L_har / V_mag;

        metricType cy_k = interp_mgr_->getCyKAero(mach, alpha_p);
        metricType xd_k = interp_mgr_->getXdKAero(alpha_p, mach);
        metricType cy_st_1deg = interp_mgr_->getCyKonsStAero(mach, 1.0 * M_PI / 180.0);
        metricType alpha_skotn = interp_mgr_->getAlphaSkOtnAero(mach, alpha_p);
        metricType mz_wz_coeff = interp_mgr_->getMzWzAero(Xm, mach);

        // Компоненты
        metricType first_comp = 0, second_comp = 0, third_comp = 0;
        for (int i = 0; i < aero_input_.rudder_count; i++) {
            metricType alpha_st_i = computeAlphaSt(alpha_p, phi_p, i);
            metricType alpha_r_i = computeAlphaR(alpha_p, phi_p, i, rudder_deflections);
            metricType phi_st_i = getPhi0St(i) + phi_p;
            metricType phi_r_i = getPhi0R(i) + phi_p;

            first_comp += interp_mgr_->getCyKonsStAero(mach, alpha_st_i) * std::sin(phi_st_i);
            second_comp += interp_mgr_->getCyKonspAero(mach, alpha_r_i, rudder_deflections[i]) * std::sin(phi_r_i);
            third_comp += interp_mgr_->getCyKonspAero(mach, alpha_r_i, rudder_deflections[i]) * std::sin(phi_r_i);
        }

        metricType L_har_inv = 1.0 / aero_input_.L_har;

        return q_dynamic * reference_area * aero_input_.L_har *
               (-cy_k * std::sin(phi_p) * (Xm - xd_k) * L_har_inv
                + first_comp * (Xm - aero_input_.Xdst) * L_har_inv
                + 1.465 * second_comp * (Xm - aero_input_.Xdp) * L_har_inv
                + mz_wz_coeff * wy_bezr
                - 2.0 * cy_st_1deg * 1.465 * third_comp * alpha_skotn * (Xm - aero_input_.Xdst) * L_har_inv);
    }

    // === МОМЕНТ РЫСКАНИЯ ===
    metricType computeMomentZ(
        metricType alpha, metricType beta, metricType alpha_p, metricType phi_p,
        metricType mach, metricType wz, metricType V_mag, metricType Xm,
        const std::vector<metricType>& rudder_deflections,
        metricType q_dynamic, metricType reference_area) const {

        metricType wz_bezr = wz * aero_input_.L_har / V_mag;

        metricType cy_k = interp_mgr_->getCyKAero(mach, alpha_p);
        metricType xd_k = interp_mgr_->getXdKAero(alpha_p, mach);
        metricType cy_st_1deg = interp_mgr_->getCyKonsStAero(mach, 1.0 * M_PI / 180.0);
        metricType alpha_skotn = interp_mgr_->getAlphaSkOtnAero(mach, alpha_p);
        metricType mz_wz_coeff = interp_mgr_->getMzWzAero(Xm, mach);

        // Компоненты
        metricType first_comp = 0, second_comp = 0, third_comp = 0;
        for (int i = 0; i < aero_input_.rudder_count; i++) {
            metricType alpha_st_i = computeAlphaSt(alpha_p, phi_p, i);
            metricType alpha_r_i = computeAlphaR(alpha_p, phi_p, i, rudder_deflections);
            metricType phi_st_i = getPhi0St(i) + phi_p;
            metricType phi_r_i = getPhi0R(i) + phi_p;

            first_comp += interp_mgr_->getCyKonsStAero(mach, alpha_st_i) * std::cos(phi_st_i);
            second_comp += interp_mgr_->getCyKonspAero(mach, alpha_r_i, rudder_deflections[i]) * std::cos(phi_r_i);
            third_comp += interp_mgr_->getCyKonspAero(mach, alpha_r_i, rudder_deflections[i]) * std::cos(phi_r_i);
        }

        metricType L_har_inv = 1.0 / aero_input_.L_har;

        return q_dynamic * reference_area * aero_input_.L_har *
               (cy_k * std::cos(phi_p) * (Xm - xd_k) * L_har_inv
                + first_comp * (Xm - aero_input_.Xdst) * L_har_inv
                + 1.465 * second_comp * (Xm - aero_input_.Xdp) * L_har_inv
                + mz_wz_coeff * wz_bezr
                - 2.0 * cy_st_1deg * 1.465 * third_comp * alpha_skotn * (Xm - aero_input_.Xdst) * L_har_inv);
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