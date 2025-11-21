//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../../../PCH.h"
#include "../../AbstractWorldModel.h"

template <typename metricType>
class SimpleWorld final : public AbstractWorldModel<metricType> {
public:
    void setWindModel(std::unique_ptr<IWindModel<metricType>> windModel) override;
    void setAtmosphericModel(std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel) override;
    void setGravityModel(std::unique_ptr<IGravitationalModel<metricType>> gravityModel) override;
    void setCoriolisEffect(std::unique_ptr<ICoriolisModel<metricType>> coriolisModel) override;
    void setTerrain(std::unique_ptr<ITerrainModel<metricType>> terrainModel) override;

private:
    std::unique_ptr<IWindModel<metricType>> windModel_;
    std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel_;
    std::unique_ptr<IGravitationalModel<metricType>> gravityModel_;
    std::unique_ptr<ICoriolisModel<metricType>> coriolisModel_;
    std::unique_ptr<ITerrainModel<metricType>> terrainModel_;
};

#include "SimpleWorld.tpp"