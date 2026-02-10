#pragma once
#include "../utils/ObjSnapshot.h"
#include "../utils/KinematicState/KinematicState.h"
#include "utils/KinematicStateDerivative/KinematicStateDerivative.h"

/**
 * \brief Вычисляет правую часть системы обыкновенных дифференциальных уравнений (ODE).
 * 
 * \tparam metricType - Тип данных для метрических величин.
 * \tparam state - Текущее кинематическое состояние.н.
 *
 * \details
 */
template <typename metricType>
class IDynamicsSystem {
public:
    virtual ~IDynamicsSystem() = default;
    
    [[nodiscard]] virtual std::string get_description() const = 0;

    virtual std::unique_ptr<KinematicStateDerivative<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) = 0;

    virtual std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& state,
        metricType t
    ) const = 0;

};