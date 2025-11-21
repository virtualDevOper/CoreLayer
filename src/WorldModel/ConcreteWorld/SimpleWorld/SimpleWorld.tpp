#pragma once

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