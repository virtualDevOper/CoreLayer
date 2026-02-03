#include "NoWindModel.h"

template <typename metricType>
Eigen::Vector3<metricType> NoWindModel<metricType>::getWindVector(Eigen::Vector3<metricType> r, metricType t) const {
    return Eigen::Vector3<metricType>::Zero();  // Используем встроенный метод Eigen для нуля
}

template <typename metricType>
metricType NoWindModel<metricType>::getWindSpeed(Eigen::Vector3<metricType> r, metricType t) const {
    return static_cast<metricType>(0);  // Явное приведение к metricType
}

template <typename metricType>
// Возвращает нулевое направление (азимут и угол возвышения)
std::pair<metricType, metricType> NoWindModel<metricType>::getWindDirection(Eigen::Vector3<metricType> r, metricType t) const {
    return {static_cast<metricType>(0), static_cast<metricType>(0)};  // Явное приведение
}

// Explicit template instantiation
#include "../../../../../include/core/GLOBAL_CONFIG.h"
template class NoWindModel<GLOBAL_CONFIG::PROJECT_TYPE>;