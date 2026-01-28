#pragma once
#include "IAeroModel.h"

template<typename metricType>
class SimpleAeroModel final : public IAeroModel<metricType> {
private:
    metricType drag_coefficient_;
    
public:
    explicit SimpleAeroModel(metricType drag_coefficient = 0.05f) 
        : drag_coefficient_(drag_coefficient) {}
    
    Eigen::Vector3<metricType> computeAerodynamicForces(
        const Eigen::Vector3<metricType>& velocity_body,
        metricType air_density,
        metricType mach_number) const override {
        
        metricType speed = velocity_body.norm();
        if (speed < 1e-6) return Eigen::Vector3<metricType>::Zero();
        return -drag_coefficient_ * speed * velocity_body;
    }
    
    Eigen::Vector3<metricType> computeAerodynamicMoments(
        const Eigen::Vector3<metricType>& velocity_body,
        const Eigen::Vector3<metricType>& angular_velocity,
        metricType air_density,
        metricType mach_number,
        const Eigen::Vector3<metricType>& euler_angles) const override {
        return Eigen::Vector3<metricType>::Zero(); // Нулевые моменты в упрощённой модели
    }
};