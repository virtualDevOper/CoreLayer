#pragma once

#include "PCH.h"

#include "../core/PCH.h"

/**
 * @file FlightMath.h
 * @brief Универсальная математическая библиотека для  расчетов.
 *
 * Эта библиотека предоставляет функции для работы с ориентацией (углы Эйлера,
 * кватернионы, матрицы поворота) и преобразованиями векторов.
 * Поддержка двух систем координат: NED (North-East-Down) и
 * ENU (East-North-Up). Пользователь может явно указать требуемую систему
 * координат для каждой операции.
 *
 */

namespace core::math {

    // Используем тип проекта из глобальной конфигурации для всех метрик.
    using metricType = GLOBAL_CONFIG::PROJECT_TYPE;
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;
    using Quaternion = Eigen::Quaterniond;

    /**
     * @brief Перечисление поддерживаемых систем координат.
     *
     * - NED (North-East-Down): Север-Восток-Вниз. Стандартная система для авиации.
     * - ENU (East-North-Up): Восток-Север-Вверх. Часто используется в геодезии и робототехнике, но мне похуй я буду ее использовать:))
     */
    enum class CoordinateFrame {
        NED,
        ENU
    };

    /**
     * @brief Перечисление поддерживаемых последовательностей углов Эйлера.
     *
     * Углы Эйлера описывают ориентацию объекта как последовательность трех поворотов.
     * - ZYX: Аэрокосмическая последовательность (рысканье -> тангаж -> крен).
     * - ZXY: Последовательность, ориентированная на БПЛА (рысканье -> крен -> тангаж).
     * - YXZ: Альтернативная последовательность.
     */
    enum class EulerConvention {
        ZYX, // Аэрокосмическая: ψ (yaw-Z), θ (pitch-Y'), φ (roll-X'')
        ZXY, // БПЛА-ориентированная: ψ (yaw-Z), φ (roll-X'), θ (pitch-Y'')
        YXZ  // Альтернативная: θ (pitch-Y), φ (roll-X'), ψ (yaw-Z'')
    };
    /*pitch - тангаж, roll - крен, yaw - рысканье*/

    /**
     * @brief Универсальная структура для хранения углов Эйлера.
     *
     * Хранит три угла и их последовательность. Не содержит информации о системе
     * координат (NED/ENU), так как это свойство контекста использования.
     */
    struct EulerAngles {
        metricType angle1; // Первый угол (зависит от convention)
        metricType angle2; // Второй угол
        metricType angle3; // Третий угол
        EulerConvention convention;

        // Конструкторы для удобства создания объектов с конкретной последовательностью.
        static EulerAngles ZYX(metricType yaw, metricType pitch, metricType roll) {
            return {yaw, pitch, roll, EulerConvention::ZYX};
        }
        static EulerAngles ZXY(metricType yaw, metricType roll, metricType pitch) {
            return {yaw, roll, pitch, EulerConvention::ZXY};
        }
        static EulerAngles YXZ(metricType pitch, metricType roll, metricType yaw) {
            return {pitch, roll, yaw, EulerConvention::YXZ};
        }
    };

    // === Вспомогательные функции для матриц элементарных поворотов ===
    namespace detail {

        inline Matrix3 Rx(metricType phi) {
            metricType c = std::cos(phi), s = std::sin(phi);
            return (Matrix3() <<
                1, 0, 0,
                0, c,-s,
                0, s, c
            ).finished();
        }

        inline Matrix3 Ry(metricType theta) {
            metricType c = std::cos(theta), s = std::sin(theta);
            return (Matrix3() <<
                c, 0, -s,    // ← БЫЛО: c, 0, s
                0, 1,  0,
                s, 0,  c     // ← БЫЛО: -s, 0, c
            ).finished();
        }

        inline Matrix3 Rz(metricType psi) {
            metricType c = std::cos(psi), s = std::sin(psi);
            return (Matrix3() <<
                c,-s, 0,
                s, c, 0,
                0, 0, 1
            ).finished();
        }

        /**
         * @brief Матрица для преобразования осей из NED в ENU и обратно.
         *
         * Эта матрица является ортогональной и сама себе обратной.
         * v_ENU = C_trans * v_NED
         * v_NED = C_trans * v_ENU
         */
        inline const Matrix3& getAxisTransformMatrix() {
            static const Matrix3 C_trans = (Matrix3() <<
                0, 1, 0,
                1, 0, 0,
                0, 0, -1
            ).finished();
            return C_trans;
        }

        /**
         * @brief Преобразует вектор из системы NED в ENU или наоборот.
         *
         * Поскольку матрица преобразования сама себе обратна, одна и та же
         * функция работает для обоих направлений.
         */
        inline Vector3 transformVector(const Vector3& v) {
            return getAxisTransformMatrix() * v;
        }

        /**
         * @brief Преобразует матрицу поворота (DCM) из системы NED в ENU или наоборот.
         *
         * Формула: C_new = C_trans * C_old * C_trans^T
         */
        inline Matrix3 transformDCM(const Matrix3& C) {
            const Matrix3& C_trans = getAxisTransformMatrix();
            return C_trans * C * C_trans.transpose();
        }

    } // namespace detail

    // === Основные преобразования ===

    // --- Углы → DCM ---
    /**
     * @brief Преобразует углы Эйлера в матрицу направляющих косинусов (DCM).
     *
     * @param e Углы Эйлера.
     * @param frame Система координат, в которой должна быть представлена DCM.
     * @return Matrix3 Матрица поворота.
     */
    inline Matrix3 eulerToDCM(const EulerAngles& e, CoordinateFrame frame = CoordinateFrame::NED) {
        using namespace detail;
        Matrix3 C;

        switch (e.convention) {
            case EulerConvention::ZYX:
                // R = R_x(φ) * R_y(θ) * R_z(ψ)
                C = Rx(e.angle3) * Ry(e.angle2) * Rz(e.angle1);
                break;
            case EulerConvention::ZXY:
                // R = R_y(θ) * R_x(φ) * R_z(ψ)
                C = Ry(e.angle3) * Rx(e.angle2) * Rz(e.angle1);
                break;
            case EulerConvention::YXZ:
                // R = R_z(ψ) * R_x(φ) * R_y(θ)
                C = Rz(e.angle3) * Rx(e.angle2) * Ry(e.angle1);
                break;
            default:
                throw std::invalid_argument("Unsupported Euler convention");
        }

        // Если требуется система координат ENU, преобразуем DCM.
        if (frame == CoordinateFrame::ENU) {
            C = transformDCM(C);
        }

        return C;
    }

    // --- DCM → углы (с обработкой гимбал-лока) ---
    /**
     * @brief Преобразует матрицу поворота (DCM) в углы Эйлера.
     *
     * @param C Матрица поворота.
     * @param conv Желаемая последовательность углов Эйлера.
     * @param frame Система координат, в которой задана входная DCM.
     * @return EulerAngles Углы Эйлера.
     */
    inline EulerAngles dcmToEuler(const Matrix3& C, EulerConvention conv, CoordinateFrame frame = CoordinateFrame::NED) {
        using namespace detail;
        constexpr metricType eps = 1e-6;

        // Если DCM задана в системе ENU, преобразуем её в NED для вычислений.
        Matrix3 C_for_calc = (frame == CoordinateFrame::ENU) ? transformDCM(C) : C;

        switch (conv) {
            case EulerConvention::ZYX: {
                metricType theta = std::asin(-C_for_calc(2,0));
                metricType phi, psi;
                if (std::abs(std::cos(theta)) > eps) {
                    phi = std::atan2(C_for_calc(2,1), C_for_calc(2,2));
                    psi = std::atan2(C_for_calc(1,0), C_for_calc(0,0));
                } else {
                    phi = 0.0;
                    psi = std::atan2(-C_for_calc(0,1), C_for_calc(1,1));
                }
                return EulerAngles::ZYX(psi, theta, phi);
            }
            case EulerConvention::ZXY: {
                metricType phi = std::asin(C_for_calc(2,1));
                metricType theta, psi;
                if (std::abs(std::cos(phi)) > eps) {
                    theta = std::atan2(-C_for_calc(2,0), C_for_calc(2,2));
                    psi = std::atan2(-C_for_calc(0,1), C_for_calc(1,1));
                } else {
                    theta = 0.0;
                    psi = std::atan2(C_for_calc(0,2), C_for_calc(0,0));
                }
                return EulerAngles::ZXY(psi, phi, theta);
            }
            case EulerConvention::YXZ: {
                metricType phi = std::asin(C_for_calc(0,2));
                metricType theta, psi;
                if (std::abs(std::cos(phi)) > eps) {
                    theta = std::atan2(-C_for_calc(1,2), C_for_calc(2,2));
                    psi = std::atan2(-C_for_calc(0,1), C_for_calc(0,0));
                } else {
                    theta = 0.0;
                    psi = std::atan2(C_for_calc(1,0), C_for_calc(1,1));
                }
                return EulerAngles::YXZ(theta, phi, psi);
            }
            default:
                throw std::invalid_argument("Unsupported Euler convention");
        }
    }

    // --- Углы → Кватернион ---
    /**
     * @brief Преобразует углы Эйлера в кватернион.
     *
     * @param e Углы Эйлера.
     * @param frame Система координат, в которой должен быть представлен кватернион.
     * @return Quaternion Нормализованный кватернион.
     */
    inline Quaternion eulerToQuaternion(const EulerAngles& e, CoordinateFrame frame = CoordinateFrame::NED) {
        Matrix3 C = eulerToDCM(e, frame);
        return Quaternion(C).normalized();
    }

    // --- Кватернион ↔ DCM ---
    /**
     * @brief Преобразует кватернион в матрицу поворота (DCM).
     *
     * @param q Кватернион.
     * @return Matrix3 Матрица поворота.
     */
    inline Matrix3 quatToDCM(const Quaternion& q) {
        return q.toRotationMatrix();
    }

    /**
     * @brief Преобразует матрицу поворота (DCM) в кватернион.
     *
     * @param C Матрица поворота.
     * @return Quaternion Нормализованный кватернион.
     */
    inline Quaternion dcmToQuaternion(const Matrix3& C) {
        return Quaternion(C).normalized();
    }

    // --- Преобразование вектора ---
    /**
     * @brief Поворачивает вектор с использованием матрицы поворота.
     *
     * @param v Вектор для поворота.
     * @param C_nb Матрица поворота, которая переводит вектор из системы тела в систему NED.
     * @param frame_of_reference Система координат, в которой должен быть результат.
     * @param body_to_frame Если true, преобразует из системы тела в неподвижную систему.
     *                      Если false, преобразует из неподвижной системы в систему тела.
     * @return Vector3 Повернутый вектор.
     */
    inline Vector3 rotateVector(const Vector3& v,
                               const Matrix3& C_nb,
                               CoordinateFrame frame_of_reference = CoordinateFrame::NED,
                               bool body_to_frame = true) {
        using namespace detail;
        Matrix3 C_nb_final = C_nb;

        // Если требуется система координат ENU, преобразуем матрицу поворота.
        if (frame_of_reference == CoordinateFrame::ENU) {
            C_nb_final = transformDCM(C_nb);
        }

        if (body_to_frame) {
            return C_nb_final * v;
        } else {
            return C_nb_final.transpose() * v;
        }
    }

    /**
     * @brief Поворачивает вектор с использованием кватерниона.
     *
     * @param v Вектор для поворота.
     * @param q Кватернион, описывающий ориентацию тела относительно NED.
     * @param frame_of_reference Система координат, в которой должен быть результат.
     * @param body_to_frame Если true, преобразует из системы тела в неподвижную систему.
     *                      Если false, преобразует из неподвижной системы в систему тела.
     * @return Vector3 Повернутый вектор.
     */
    inline Vector3 rotateVector(const Vector3& v,
                               const Quaternion& q,
                               CoordinateFrame frame_of_reference = CoordinateFrame::NED,
                               bool body_to_frame = true) {
        using namespace detail;
        Quaternion q_final = q;

        // Если требуется система координат ENU, нужно преобразовать кватернион.
        // Кватернион преобразуется путем применения преобразования к его векторной части.
        if (frame_of_reference == CoordinateFrame::ENU) {
            Vector3 vec_part = q.vec();
            vec_part = transformVector(vec_part);
            q_final = Quaternion(q.w(), vec_part.x(), vec_part.y(), vec_part.z()).normalized();
        }

        if (body_to_frame) {
            return q_final * v;
        } else {
            return q_final.conjugate() * v;
        }
    }

    // --- Кинематика: [φ̇, θ̇, ψ̇] → [p, q, r] ---
    /**
     * @brief Преобразует производные углов Эйлера в угловую скорость в системе тела.
     *
     * @param e Текущие углы Эйлера.
     * @param rates Вектор производных углов [φ̇, θ̇, ψ̇].
     * @param frame Система координат, в которой заданы производные углов.
     * @return Vector3 Угловая скорость в системе тела [p, q, r].
     */
    inline Vector3 eulerRatesToAngularVelocity(const EulerAngles& e,
                                              const Vector3& rates,
                                              CoordinateFrame frame = CoordinateFrame::NED) {
        using namespace detail;
        metricType a1 = e.angle1, a2 = e.angle2, a3 = e.angle3;
        Matrix3 T;

        // Вычисляем матрицу T в предположении, что углы заданы в системе NED.
        switch (e.convention) {
            case EulerConvention::ZYX: {
                metricType s3 = std::sin(a3), c3 = std::cos(a3); // roll
                metricType s2 = std::sin(a2), c2 = std::cos(a2); // pitch
                T <<
                    1, 0, -s2,
                    0, c3, s3*c2,
                    0, -s3, c3*c2;
                break;
            }
            case EulerConvention::ZXY: {
                metricType s2 = std::sin(a2), c2 = std::cos(a2); // roll
                metricType s3 = std::sin(a3), c3 = std::cos(a3); // pitch
                T <<
                    c3, s2*s3, 0,
                    0, c2, -1,
                    s3, -s2*c3, 0;
                break;
            }
            case EulerConvention::YXZ: {
                metricType s2 = std::sin(a2), c2 = std::cos(a2); // roll
                metricType s1 = std::sin(a1), c1 = std::cos(a1); // pitch
                T <<
                    c1, s1*s2, 0,
                    0, c2, -1,
                    -s1, c1*s2, 0;
                break;
            }
            default:
                throw std::invalid_argument("Unsupported Euler convention for kinematics");
        }

        // Вычисляем угловую скорость в системе NED.
        Vector3 omega_ned = T * rates;

        // Если исходные производные были заданы в ENU, то и результат должен быть в ENU.
        // Угловая скорость преобразуется как обычный вектор.
        return (frame == CoordinateFrame::ENU) ? transformVector(omega_ned) : omega_ned;
    }

    // --- Кинематика: [p, q, r] → [φ̇, θ̇, ψ̇] (с проверкой сингулярности) ---
    /**
     * @brief Преобразует угловую скорость в системе тела в производные углов Эйлера.
     *
     * @param e Текущие углы Эйлера.
     * @param omega Угловая скорость в системе тела [p, q, r].
     * @param frame Система координат, в которой должен быть результат.
     * @return Vector3 Вектор производных углов [φ̇, θ̇, ψ̇].
     */
    inline Vector3 angularVelocityToEulerRates(const EulerAngles& e,
                                              const Vector3& omega,
                                              const CoordinateFrame frame = CoordinateFrame::NED) {
        using namespace detail;
        constexpr metricType eps = 1e-6;
        metricType a1 = e.angle1;
        const metricType a2 = e.angle2;
        const metricType a3 = e.angle3;
        Matrix3 T_inv;

        // Если угловая скорость задана в системе ENU, преобразуем её в NED для вычислений.
        const Vector3 omega_for_calc = (frame == CoordinateFrame::ENU) ? transformVector(omega) : omega;

        // Вычисляем обратную матрицу T_inv в предположении, что углы заданы в системе NED.
        switch (e.convention) {
            case EulerConvention::ZYX: {
                const metricType s3 = std::sin(a3);
                const metricType c3 = std::cos(a3);
                metricType s2 = std::sin(a2);
                const metricType c2 = std::cos(a2);
                if (std::abs(c2) < eps) throw std::runtime_error("Gimbal lock in ZYX at pitch = ±90°");
                const metricType t = 1.0 / c2;
                T_inv <<
                    1, s3*t,  c3*t,
                    0, c3,   -s3,
                    0, s3*t, c3*t;
                break;
            }
            case EulerConvention::ZXY: {
                const metricType s2 = std::sin(a2);
                const metricType c2 = std::cos(a2);
                if (std::abs(c2) < eps) throw std::runtime_error("Gimbal lock in ZXY at roll = ±90°");
                const metricType t = 1.0 / c2;
                T_inv <<
                    1/c2, 0, s2*t,
                    0,    1, 0,
                    -s2*t,0, c2*t;
                break;
            }
            case EulerConvention::YXZ: {
                const metricType s2 = std::sin(a2);
                const metricType c2 = std::cos(a2);
                if (std::abs(c2) < eps) throw std::runtime_error("Gimbal lock in YXZ at roll = ±90°");
                const metricType t = 1.0 / c2;
                T_inv <<
                    1/c2, 0, -s2*t,
                    0,    1, 0,
                    s2*t, 0, c2*t;
                break;
            }
            default:
                throw std::invalid_argument("Unsupported Euler convention for inverse kinematics");
        }

        // Вычисляем производные углов в системе NED.
        Vector3 rates_ned = T_inv * omega_for_calc;

        // Если требуется результат в системе ENU, преобразуем вектор производных.
        return (frame == CoordinateFrame::ENU) ? transformVector(rates_ned) : rates_ned;
    }

    // === Обратная совместимость ===
    // Старые функции перенаправляют вызовы на новые, используя NED по умолчанию.

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Matrix3 eulerToDCM(const EulerAngles& e) {
        return eulerToDCM(e, CoordinateFrame::NED);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline EulerAngles dcmToEuler(const Matrix3& C, EulerConvention conv) {
        return dcmToEuler(C, conv, CoordinateFrame::NED);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Quaternion eulerToQuaternion(const EulerAngles& e) {
        return eulerToQuaternion(e, CoordinateFrame::NED);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Vector3 rotateVector(const Vector3& v, const Matrix3& C_nb, bool body_to_ned) {
        return rotateVector(v, C_nb, CoordinateFrame::NED, body_to_ned);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Vector3 rotateVector(const Vector3& v, const Quaternion& q, bool body_to_ned) {
        return rotateVector(v, q, CoordinateFrame::NED, body_to_ned);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Vector3 eulerRatesToAngularVelocity(const EulerAngles& e, const Vector3& rates) {
        return eulerRatesToAngularVelocity(e, rates, CoordinateFrame::NED);
    }

    [[deprecated("Используйте версию с параметром CoordinateFrame. По умолчанию используется NED.")]]
    inline Vector3 angularVelocityToEulerRates(const EulerAngles& e, const Vector3& omega) {
        return angularVelocityToEulerRates(e, omega, CoordinateFrame::NED);
    }

} // namespace core::math
