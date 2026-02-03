#include "PCH.h"
#include "KavendishModel.h"

template <typename metricType>
metricType KavendishModel<metricType>::getGravitationalAcceleration(const Eigen::Vector3<metricType> r) const {
    auto h = r.z();
    return (PhysicsConstants::gravitationalConstant * PhysicsConstants::earthMass) / 
           pow(h + PhysicsConstants::earthRadius, 2);
}

// Explicit template instantiation
template class KavendishModel<GLOBAL_CONFIG::PROJECT_TYPE>;