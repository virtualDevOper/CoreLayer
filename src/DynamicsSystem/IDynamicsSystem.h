//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../utils/ObjSnapshot.h"

template <typename metricType>
class IDynamicsSystem {
public:
    virtual ~IDynamicsSystem() = default;
    virtual std::string get_description() = 0;
    virtual std::unique_ptr<ObjSnapshot<metricType>> get_rhs_derivatives(
        const ObjSnapshot<metricType>& previous_state,
        metricType t) = 0;
};