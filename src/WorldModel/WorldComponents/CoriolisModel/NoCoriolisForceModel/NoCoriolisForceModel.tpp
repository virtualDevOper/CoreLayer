//
// Created by 4NR_Operator_3 on 17.09.2025.

#pragma once

template <typename metricType>
    Eigen::Vector3<metricType> NoCoriolisForceModel<metricType>::getCoriolisForce(const Eigen::Vector3<metricType> r) const{
        return Eigen::Vector3<metricType>::Zero(); ;
    };
