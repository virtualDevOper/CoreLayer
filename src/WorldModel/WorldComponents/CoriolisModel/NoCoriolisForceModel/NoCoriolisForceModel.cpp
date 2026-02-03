//
// Created by 4NR_Operator_3 on 17.09.2025.

#include "PCH.h"
#include "NoCoriolisForceModel.h"

template <typename metricType>
Eigen::Vector3<metricType> NoCoriolisForceModel<metricType>::getCoriolisForce(const Eigen::Vector3<metricType> r) const {
    return Eigen::Vector3<metricType>::Zero();
}

// Explicit template instantiation for common types
template class NoCoriolisForceModel<GLOBAL_CONFIG::PROJECT_TYPE>;