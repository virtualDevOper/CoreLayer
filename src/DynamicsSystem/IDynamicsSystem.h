#pragma once
#include "../utils/ObjSnapshot.h"
#include "../utils/KinematicState.h"

template <typename metricType>
class IDynamicsSystem {
public:
    virtual ~IDynamicsSystem() = default;
    virtual std::string get_description() = 0;

    // === ИЗМЕНЕНА СИГНАТУРА: работает ТОЛЬКО с кинематикой ===
    virtual std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) = 0;

    // === ИЗМЕНЕНА СИГНАТУРА: чистая функция, возвращает НОВЫЙ снапшот ===
    virtual std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) const = 0;
};