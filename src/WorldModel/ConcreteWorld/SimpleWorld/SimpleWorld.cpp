#include "SimpleWorld.h"

template <typename metricType>
void SimpleWorld<metricType>::setWindModel(std::unique_ptr<IWindModel<metricType>> windModel) {
    windModel_ = std::move(windModel);
}

template <typename metricType>
void SimpleWorld<metricType>::setAtmosphericModel(std::unique_ptr<IAtmosphericModel<metricType>> atmosphericModel) {
    atmosphericModel_ = std::move(atmosphericModel);
}

template <typename metricType>
void SimpleWorld<metricType>::setGravityModel(std::unique_ptr<IGravitationalModel<metricType>> gravityModel) {
    gravityModel_ = std::move(gravityModel);
}

template <typename metricType>
void SimpleWorld<metricType>::setCoriolisEffect(std::unique_ptr<ICoriolisModel<metricType>> coriolisModel) {
    coriolisModel_ = std::move(coriolisModel);
}

template <typename metricType>
void SimpleWorld<metricType>::setTerrain(std::unique_ptr<ITerrainModel<metricType>> terrainModel) {
    terrainModel_ = std::move(terrainModel);
}

template <typename metricType>
const IWindModel<metricType>* SimpleWorld<metricType>::getWindModel() const {
    return windModel_.get();
}

template <typename metricType>
const IAtmosphericModel<metricType>* SimpleWorld<metricType>::getAtmosphericModel() const {
    return atmosphericModel_.get();
}

template <typename metricType>
const IGravitationalModel<metricType>* SimpleWorld<metricType>::getGravityModel() const {
    return gravityModel_.get();
}

template <typename metricType>
const ICoriolisModel<metricType>* SimpleWorld<metricType>::getCoriolisModel() const {
    return coriolisModel_.get();
}

template <typename metricType>
const ITerrainModel<metricType>* SimpleWorld<metricType>::getTerrainModel() const {
    return terrainModel_.get();
}

template <typename metricType>
bool SimpleWorld<metricType>::hasWindModel() const {
    return windModel_ != nullptr;
}

template <typename metricType>
bool SimpleWorld<metricType>::hasAtmosphericModel() const {
    return atmosphericModel_ != nullptr;
}

template <typename metricType>
bool SimpleWorld<metricType>::hasGravityModel() const {
    return gravityModel_ != nullptr;
}

template <typename metricType>
bool SimpleWorld<metricType>::hasCoriolisModel() const {
    return coriolisModel_ != nullptr;
}

template <typename metricType>
bool SimpleWorld<metricType>::hasTerrainModel() const {
    return terrainModel_ != nullptr;
}

// Explicit template instantiation
#include "../../../../include/core/GLOBAL_CONFIG.h"
template class SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>;