#pragma once
#include "../IDynamicsSystem.h"
#include "../../utils/ConeDirection/ConeDirection.h"
#include "../../utils/DynamicParametersProviderForFullRocketModel.h"
#include "../../WorldModel/AbstractWorldModel.h"

/**
 * \brief Full 6DOF rigid body dynamics system for rockets/aircraft.
 *
 * \tparam metricType Numeric type for calculations.
 *
 * \details Implements complete equations of motion including coordinate frame
 * transformations, aerodynamic forces/moments, gravity, and inertial effects.
 * Used by RungeKutta4Solver for trajectory integration.
 */
template <typename metricType>
class FullRocketODE final : public IDynamicsSystem<metricType> {
private:
    std::weak_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider_;
    std::weak_ptr<AbstractWorldModel<metricType>> world_;
    // Cache last computed forces and moments for snapshot augmentation
    mutable Eigen::Vector3<metricType> last_computed_forces_;
    mutable Eigen::Vector3<metricType> last_computed_moments_;

public:
    explicit FullRocketODE(
        const std::shared_ptr<DynamicParametersProviderForFullRocketModel<metricType>> params_provider,
        const std::shared_ptr<AbstractWorldModel<metricType>> world
    );

    std::string get_description() const override;

    std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) override;

    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) const override;

private:
    // Coordinate frame transformations
    Eigen::Vector3<metricType> transformVelocityEarthToBody(
        const Eigen::Vector3<metricType>& V_earth,
        const Eigen::Vector3<metricType>& euler) const;

    Eigen::Vector3<metricType> transformAccelerationBodyToEarth(
        const Eigen::Vector3<metricType>& a_body,
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& euler) const;

    // Physics calculations
    metricType computeSpaceAngleOfAttack(const Eigen::Vector3<metricType>& V_body) const;
    metricType computeAerodynamicRollAngle(const Eigen::Vector3<metricType>& V_body) const;
    metricType computeMachNumber(metricType V_magnitude, const Eigen::Vector3<metricType>& position) const;
    metricType computeAirDensity(const Eigen::Vector3<metricType>& position) const;
    
    Eigen::Vector3<metricType> computeTotalForces(
        const Eigen::Vector3<metricType>& F_aero,
        const Eigen::Vector3<metricType>& F_thrust,
        metricType mass,
        const Eigen::Vector3<metricType>& euler) const;

    Eigen::Vector3<metricType> computeLinearAccelerationBody(
        const Eigen::Vector3<metricType>& V_body,
        const Eigen::Vector3<metricType>& omega,
        const Eigen::Vector3<metricType>& F_sum,
        metricType mass) const;

    Eigen::Vector3<metricType> computeEulerAnglesDerivatives(
        const Eigen::Vector3<metricType>& euler,
        const Eigen::Vector3<metricType>& omega) const;

    Eigen::Vector3<metricType> computeAngularAccelerationBody(
        const Eigen::Vector3<metricType>& M_sum,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& omega) const;
};