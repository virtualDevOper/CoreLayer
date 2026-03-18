#include "PCH.h"
#include "NetAeroModel.h"
#include "CONSTANTS.h"


template<typename metricType>
Eigen::Vector3<metricType> NetAeroModel<metricType>::computeForceENU(
    const Eigen::Vector3<metricType>& V_enu,
    metricType rho,
    metricType t
) const {
    metricType V_mag = V_enu.norm();
    if (V_mag < metricType(1e-8)) {
        return Eigen::Vector3<metricType>::Zero();
    }

    // Интерполяция угла атаки (для 3DOF берём 0, но можно доработать)
    metricType alpha_deg = metricType(0);

    // Получаем Cx и S_ref с учётом фазы раскрытия
    metricType cx = getCx(alpha_deg, t);
    metricType s_ref = getSRef(t);

    metricType q_dyn = metricType(0.5) * rho * V_mag * V_mag;
    metricType drag_magnitude = cx * q_dyn * s_ref;

    // Сила направлена против вектора скорости
    return -drag_magnitude * (V_enu / V_mag);
}

// Явная инстанциация шаблона
template class NetAeroModel<GLOBAL_CONFIG::PROJECT_TYPE>;