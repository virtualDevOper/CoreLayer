//
// Created by 4NR_Operator_3 on 17.09.2025.
//
#pragma once

template <typename metricType>
metricType PlaneTerrain<metricType>::getHeight(metricType x, metricType y) const{
    return static_cast<metricType>(0.0);
};

template <typename metricType>
bool PlaneTerrain<metricType>::hasHeightMap() const{
    return false;
};
