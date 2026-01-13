//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "../../PCH.h"
#include "WorldComponents/AtmosphericModel/IAtmosphericModel.h"
#include "WorldComponents/CoriolisModel/ICoriolisModel.h"
#include "WorldComponents/WindModel/IWindModel.h"
#include "WorldComponents/GravityModel/IGravitationalModel.h"
#include "WorldComponents/ITerrainModel/ITerrainModel.h"

template <typename metricType>
class AbstractWorldModel {
public:
    virtual ~AbstractWorldModel() = default;

    virtual void setWindModel(std::unique_ptr<IWindModel<metricType>> windModel) = 0;
    virtual void setAtmosphericModel(std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel) = 0;
    virtual void setGravityModel(std::unique_ptr<IGravitationalModel<metricType>> gravityModel) = 0;
    virtual void setCoriolisEffect(std::unique_ptr<ICoriolisModel<metricType>> coriolisModel) = 0;
    virtual void setTerrain(std::unique_ptr<ITerrainModel<metricType>> terrainModel) = 0;

    virtual IWindModel<metricType>* getWindModel() const = 0;
    virtual IAtmosphericModel<metricType>* getAtmosphericModel() const = 0;
    virtual IGravitationalModel<metricType>* getGravityModel() const = 0;
    virtual ICoriolisModel<metricType>* getCoriolisModel() const = 0;
    virtual ITerrainModel<metricType>* getTerrainModel() const = 0;

    virtual bool hasWindModel() const = 0;
    virtual bool hasAtmosphericModel() const = 0;
    virtual bool hasGravityModel() const = 0;
    virtual bool hasCoriolisModel() const = 0;
    virtual bool hasTerrainModel() const = 0;

    /* TODO
    virtual void setInteractionSystem(std::unique_ptr<IInteractionSystem> interactionSystem) = 0;
    virtual IInteractionSystem* getInteractionSystem() const = 0;
    */
};