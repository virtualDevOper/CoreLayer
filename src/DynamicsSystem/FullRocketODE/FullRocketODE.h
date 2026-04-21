#pragma once
#include "../IDynamicsSystem.h"
#include "../../../include/math/FlightMath.h"
#include "../../utils/ParameterProvider/DynamicParametersProvider.h"
#include "../../WorldModel/AbstractWorldModel.h"
#include "../../../include/aero_simpi/aerodynamics.h"

/**
 * СОГЛАШЕНИЕ у меня тут такое (ENU - East-North-Up):
 *
 * Earth frame (ENU):
 *   X_e → East  (горизонтально вправо)
 *   Y_e → North (горизонтально вперёд)
 *   Z_e ↑ Up    (вертикально вверх) ← КЛЮЧЕВОЙ ПРИЗНАК ENU в отличии от NED
 *   Gravity: (0, 0, -mg)
 *
 * Body frame (связанная СК ракеты):
 *   X_b → Nose (нос ракеты)
 *   Y_b → Right wing (правый борт)
 *   Z_b ↑ Up
 *
 * Конвенция углов: [psi, theta, gamma]
 *   euler[0] = psi   (yaw/курс)   — поворот вокруг Z
 *   euler[1] = theta (pitch/тангаж) — поворот вокруг Y'
 *   euler[2] = gamma (roll/крен)   — поворот вокруг X''
 *
 * Последовательность поворота: 3-2-1 (ZYX)
 *   R_body_to_earth = Rz(psi) · Ry(theta) · Rx(gamma)
 */

/**
 * \brief Полная 6DOF (6 степеней свободы ) система ОДУ для ЗУР (но по идее подо все можно переделать).
 *
 * \tparam metricType Тип данных для метрических величин.
 *
* \details Реализует полные уравнения движения по старым книжкам дедов (не про армейских дедов, а реальных дедов{старых типов}).
* В общем, в падлу расписывать - смотри функцию `get_rhs_derivatives`, там все достаточно просто
*/
template <typename metricType>
class FullRocketODE final : public IDynamicsSystem<metricType> {
private:
    std::shared_ptr<DynamicParametersProvider<metricType>> params_provider_;
    std::weak_ptr<AbstractWorldModel<metricType>> world_;
    std::shared_ptr<aero::AerodynamicsModel> aero_model_;
    mutable Eigen::Vector3<metricType> last_computed_forces_;
    mutable Eigen::Vector3<metricType> last_computed_moments_;
    mutable aero::AeroOutput last_aero_output_;
    mutable metricType last_alpha_;
    mutable metricType last_beta_;
    mutable metricType last_mach_;

public:
    explicit FullRocketODE(
        std::shared_ptr<DynamicParametersProvider<metricType>> params_provider,
        std::shared_ptr<AbstractWorldModel<metricType>> world,
        std::shared_ptr<aero::AerodynamicsModel> aero_model
    );

    std::string get_description() const override;

    std::unique_ptr<KinematicStateDerivative<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) override;

    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& state,
        metricType t
    ) const override;

private:
    metricType computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const;
    metricType computeAerodynamicRollAngle(const Eigen::Vector3<metricType>& V_body) const;
    metricType computeSideslipAngle(const Eigen::Vector3<metricType>& V_body) const;
    metricType computeMachNumber(metricType V_magnitude, const Eigen::Vector3<metricType>& position) const;
    metricType computeAirDensity(const Eigen::Vector3<metricType>& position) const;

    Eigen::Vector3<metricType> computeTotalForcesENU(
        const Eigen::Vector3<metricType>& F_aero_body,
        const Eigen::Vector3<metricType>& F_thrust_body,
        metricType mass,
        const core::math::Matrix3& dcm) const;

    Eigen::Vector3<metricType> computeLinearAccelerationENU(
        const Eigen::Vector3<metricType>& F_sum,
        metricType mass);

    Eigen::Vector3<metricType> computeEulerAnglesDerivatives(
        const Eigen::Vector3<metricType>& euler,
        const Eigen::Vector3<metricType>& angularVelocity) const;

    Eigen::Vector3<metricType> computeAngularAccelerationBody(
        const Eigen::Vector3<metricType>& M_sum,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& angularVelocity) const;
};