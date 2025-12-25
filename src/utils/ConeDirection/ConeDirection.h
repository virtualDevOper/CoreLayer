//
// Created by 4NR_Operator_3 on 25.12.2025.
//

//
// Created by 4NR_Operator_3 on 25.12.2025
// Template version of coordinate transformations
// REFACTORED: Smart pointers everywhere
//

#pragma once
#include "../../../PCH.h"

/**
 * \brief Интерфейс для трансформаций между системами координат
 *
 * Соглашение углов Эйлера: [theta, psi, gamma] = [тангаж, рыскание, крен]
 */
template <typename metricType>
class ConeDirectionInterface {
public:
    virtual ~ConeDirectionInterface() = default;

    std::array<metricType, 3> result_getter() const {
        return result;
    }

protected:
    ConeDirectionInterface(metricType theta, metricType psi, metricType gamma,
                          const std::array<metricType, 3>& vector_param)
        : theta(theta), gamma(gamma), psi(psi) {}

    virtual std::array<std::array<metricType, 3>, 3> rotation_matrix() = 0;

    std::array<metricType, 3> result = {0, 0, 0};
    metricType theta;   // тангаж
    metricType psi;     // рыскание
    metricType gamma;   // крен
};

/**
 * \brief Трансформация из связной СК ЛА в земную СК
 *
 * V_earth = L * V_body
 */
template <typename metricType>
class Svyaz_to_zemn_Direction : public ConeDirectionInterface<metricType> {
public:
    Svyaz_to_zemn_Direction(metricType theta, metricType psi, metricType gamma,
                           const std::array<metricType, 3>& vector_param)
        : ConeDirectionInterface<metricType>(theta, psi, gamma, vector_param) {

        auto L = this->rotation_matrix();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                this->result[i] += L[i][j] * vector_param[j];
            }
        }
    }

    virtual ~Svyaz_to_zemn_Direction() = default;

private:
    std::array<std::array<metricType, 3>, 3> rotation_matrix() override {
        metricType cos_psi = std::cos(this->psi);
        metricType sin_psi = std::sin(this->psi);
        metricType cos_theta = std::cos(this->theta);
        metricType sin_theta = std::sin(this->theta);
        metricType cos_gamma = std::cos(this->gamma);
        metricType sin_gamma = std::sin(this->gamma);

        return { {
            {
                cos_psi * cos_theta,
                -cos_psi * sin_theta * cos_gamma + sin_psi * sin_gamma,
                cos_psi * sin_theta * sin_gamma + sin_psi * cos_gamma
            },
            {
                sin_theta,
                cos_theta * cos_gamma,
                -cos_theta * sin_gamma
            },
            {
                -cos_theta * sin_psi,
                cos_psi * sin_gamma + sin_theta * sin_psi * cos_gamma,
                cos_psi * cos_gamma - sin_theta * sin_psi * sin_gamma
            }
        }};
    }
};

/**
 * \brief Трансформация из земной СК в связную СК ЛА
 *
 * V_body = L^T * V_earth
 */
template <typename metricType>
class Zemn_to_svyaz_Direction : public ConeDirectionInterface<metricType> {
public:
    Zemn_to_svyaz_Direction(metricType theta, metricType psi, metricType gamma,
                           const std::array<metricType, 3>& vector_param)
        : ConeDirectionInterface<metricType>(theta, psi, gamma, vector_param) {

        auto L = this->rotation_matrix();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                this->result[i] += L[i][j] * vector_param[j];
            }
        }
    }

    virtual ~Zemn_to_svyaz_Direction() = default;

private:
    std::array<std::array<metricType, 3>, 3> rotation_matrix() override {
        metricType cos_psi = std::cos(this->psi);
        metricType sin_psi = std::sin(this->psi);
        metricType cos_theta = std::cos(this->theta);
        metricType sin_theta = std::sin(this->theta);
        metricType cos_gamma = std::cos(this->gamma);
        metricType sin_gamma = std::sin(this->gamma);

        return { {
            {
                cos_psi * cos_theta,
                sin_theta,
                -cos_theta * sin_psi
            },
            {
                -cos_psi * sin_theta * cos_gamma + sin_psi * sin_gamma,
                cos_theta * cos_gamma,
                cos_psi * sin_gamma + sin_theta * sin_psi * cos_gamma
            },
            {
                cos_psi * sin_theta * sin_gamma + sin_psi * cos_gamma,
                -cos_theta * sin_gamma,
                cos_psi * cos_gamma - sin_theta * sin_psi * sin_gamma
            }
        }};
    }
};

/**
 * \brief Factory для создания трансформаций
 */
template <typename metricType>
class TransformationFactory {
public:
    static std::unique_ptr<Svyaz_to_zemn_Direction<metricType>>
    createBodyToEarthTransform(
        metricType theta, metricType psi, metricType gamma,
        const std::array<metricType, 3>& vector) {
        return std::make_unique<Svyaz_to_zemn_Direction<metricType>>(
            theta, psi, gamma, vector);
    }

    static std::unique_ptr<Zemn_to_svyaz_Direction<metricType>>
    createEarthToBodyTransform(
        metricType theta, metricType psi, metricType gamma,
        const std::array<metricType, 3>& vector) {
        return std::make_unique<Zemn_to_svyaz_Direction<metricType>>(
            theta, psi, gamma, vector);
    }
};
