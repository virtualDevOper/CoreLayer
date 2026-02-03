#pragma once
#include "../utils/ObjSnapshot.h"
#include "../utils/KinematicState.h"

/**
 * \brief Interface for object dynamics systems.
 * 
 * \tparam metricType Numeric type for calculations.
 * 
 * \details Defines the contract for all dynamics systems in the simulator.
 * Encapsulates computation of differential equation right-hand sides
 * and augmentation of kinematic states with physical parameters.
 * Used by integrators for step-by-step trajectory calculation.
 */
template <typename metricType>
class IDynamicsSystem {
public:
    virtual ~IDynamicsSystem() = default;
    
    virtual std::string get_description() const = 0;

    // Compute derivatives for kinematic state only
    virtual std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) = 0;

    // Pure function: returns NEW snapshot with additional physical parameters
    virtual std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) const = 0;
};