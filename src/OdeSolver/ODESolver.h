//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "../../PCH.h"
#include "../PhysicalObjects/ObjectManager.h"
#include "../SimulationMomento/SimulationMomento.h"

//TODO
// === Реализовать проверку коллизий между объектами и с землей ===

template <typename metricType, typename CallbackType>
class ODESolver {
public:
    virtual ~ODESolver() = default;

    virtual void solve(
        std::shared_ptr<ObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        std::shared_ptr<SimulationMomento<metricType>> momento
    ) = 0;

protected:
    std::vector<int> checkCollisions(const std::vector<StateStorage<metricType>>& stateStorages) {
        // TODO: Реализовать проверку коллизий
        return {};
    }
};