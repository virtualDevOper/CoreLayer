//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once


template <typename metricType>
metricType KavendishModel<metricType>::getGravitationalAcceleration(const Eigen::Vector3<metricType> r) const{
    auto h = r.z();
    return (PhysicsConstants::gravitationalConstant * PhysicsConstants::earthMass)/pow(h + PhysicsConstants::earthRadius,2);
};


