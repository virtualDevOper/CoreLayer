#pragma once
#include "../AbstractObject.h"
#include "../../DynamicsSystem/SimpleKinematicsSystem/SimpleKinematicsSystem.h"

template<typename metricType>
class SimpleObject final : public AbstractObject<metricType> {
public:
    SimpleObject(
        const Eigen::Vector3<metricType>& initial_position,
        const Eigen::Vector3<metricType>& initial_velocity
    );

private:
    static std::unique_ptr<ObjInitParams<metricType>> createSimpleInitParams(
        const Eigen::Vector3<metricType>& position,
        const Eigen::Vector3<metricType>& velocity
    );

};