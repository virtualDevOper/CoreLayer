//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once
#include "../ITerrainModel.h"

template <typename metricType>
class PlaneTerrain final: public ITerrainModel<metricType>{
public:
    metricType getHeight(metricType x, metricType y) const override;
    [[nodiscard]] bool hasHeightMap() const override;
};
