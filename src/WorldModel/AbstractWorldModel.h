#pragma once
#include "PCH.h"
#include "WorldComponents/AtmosphericModel/IAtmosphericModel.h"
#include "WorldComponents/CoriolisModel/ICoriolisModel.h"
#include "WorldComponents/WindModel/IWindModel.h"
#include "WorldComponents/GravityModel/IGravitationalModel.h"
#include "WorldComponents/ITerrainModel/ITerrainModel.h"

/**
 * \brief Abstract physical world model for simulation.
 *
 * \tparam metricType Numeric type for calculations.
 *
 * \details Aggregates all physical environment components: atmosphere, wind,
 * gravity, Coriolis forces, and terrain. Provides unified interface for
 * accessing environmental models. Supports dynamic component replacement
 * and availability checking. Used by dynamics systems to calculate
 * external forces and effects.
 */
template <typename metricType>
class AbstractWorldModel {
public:
    virtual ~AbstractWorldModel() = default;

    // Component setters
    virtual void setWindModel(std::unique_ptr<IWindModel<metricType>> windModel) = 0;
    virtual void setAtmosphericModel(std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel) = 0;
    virtual void setGravityModel(std::unique_ptr<IGravitationalModel<metricType>> gravityModel) = 0;
    virtual void setCoriolisEffect(std::unique_ptr<ICoriolisModel<metricType>> coriolisModel) = 0;
    virtual void setTerrain(std::unique_ptr<ITerrainModel<metricType>> terrainModel) = 0;

    // Component getters - const to prevent modification
    virtual const IWindModel<metricType>* getWindModel() const = 0;
    virtual const IAtmosphericModel<metricType>* getAtmosphericModel() const = 0;
    virtual const IGravitationalModel<metricType>* getGravityModel() const = 0;
    virtual const ICoriolisModel<metricType>* getCoriolisModel() const = 0;
    virtual const ITerrainModel<metricType>* getTerrainModel() const = 0;

    // Component availability checks
    [[nodiscard]] virtual bool hasWindModel() const = 0;
    [[nodiscard]] virtual bool hasAtmosphericModel() const = 0;
    [[nodiscard]] virtual bool hasGravityModel() const = 0;
    [[nodiscard]] virtual bool hasCoriolisModel() const = 0;
    [[nodiscard]] virtual bool hasTerrainModel() const = 0;

    /* TODO: Interaction system support
    virtual void setInteractionSystem(std::unique_ptr<IInteractionSystem> interactionSystem) = 0;
    virtual IInteractionSystem* getInteractionSystem() const = 0;
    */
};