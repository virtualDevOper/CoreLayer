//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "PCH.h"
#include "../../AbstractWorldModel.h"

template <typename metricType>
class SimpleWorld final : public AbstractWorldModel<metricType> {
public:
    SimpleWorld() = default;
    ~SimpleWorld() override = default;

    // Delete copy operations (RAII for unique_ptr)
    SimpleWorld(const SimpleWorld&) = delete;
    SimpleWorld& operator=(const SimpleWorld&) = delete;

    // Allow move operations
    SimpleWorld(SimpleWorld&&) noexcept = default;
    SimpleWorld& operator=(SimpleWorld&&) noexcept = default;

    // Setters
    void setWindModel(std::unique_ptr<IWindModel<metricType>> windModel) override;
    void setAtmosphericModel(std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel) override;
    void setGravityModel(std::unique_ptr<IGravitationalModel<metricType>> gravityModel) override;
    void setCoriolisEffect(std::unique_ptr<ICoriolisModel<metricType>> coriolisModel) override;
    void setTerrain(std::unique_ptr<ITerrainModel<metricType>> terrainModel) override;

    // Getters - возвращают const указатели для предотвращения модификации
    const IWindModel<metricType>* getWindModel() const override;
    const IAtmosphericModel<metricType>* getAtmosphericModel() const override;
    const IGravitationalModel<metricType>* getGravityModel() const override;
    const ICoriolisModel<metricType>* getCoriolisModel() const override;
    const ITerrainModel<metricType>* getTerrainModel() const override;

    // Checking methods
    bool hasWindModel() const override;
    bool hasAtmosphericModel() const override;
    bool hasGravityModel() const override;
    bool hasCoriolisModel() const override;
    bool hasTerrainModel() const override;

private:
    std::unique_ptr<IWindModel<metricType>> windModel_;
    std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel_;
    std::unique_ptr<IGravitationalModel<metricType>> gravityModel_;
    std::unique_ptr<ICoriolisModel<metricType>> coriolisModel_;
    std::unique_ptr<ITerrainModel<metricType>> terrainModel_;
};