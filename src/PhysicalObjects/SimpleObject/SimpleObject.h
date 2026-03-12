// src/PhysicalObjects/SimpleObject/SimpleObject.h
#pragma once
#include "../AbstractObject.h"
#include "../../DynamicsSystem/SimpleKinematicsSystem/SimpleKinematicsSystem.h"
#include <optional>

template<typename metricType>
class SimpleObject final : public AbstractObject<metricType> {
public:
    // === КОНСТРУКТОР С НАЧАЛЬНЫМИ УСЛОВИЯМИ ===
    SimpleObject(
        const Eigen::Vector3<metricType>& initial_position,
        const Eigen::Vector3<metricType>& initial_velocity,
        metricType mass = metricType(0)           // Опционально
    );

    // === КОНТРОЛЬ КОЛЛИЗИЙ ===
    void setCollisionThreshold(metricType distance);  // Дистанция "взрыва"
    std::optional<int> checkCollision(const AbstractObject<metricType>* other,
                                      int other_id);   // Возвращает ID при коллизии

    [[nodiscard]] bool hasCollisionBeenDetected() const noexcept;

private:
    static std::unique_ptr<ObjInitParams<metricType>> createSimpleInitParams(
        const Eigen::Vector3<metricType>& position,
        const Eigen::Vector3<metricType>& velocity
    );

    metricType collision_threshold_{metricType{0}};
    bool collision_detected_{false};
};