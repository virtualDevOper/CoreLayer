#pragma once

#include "../AeroInput/RocketAeroInput.h"
#include "../IAeroModel.h"
#include "../../../../utils/Interpolation/ComponentInterpolationManager.h"
#include "FullAeroModel.h"

/**
 * \brief Адаптер полной аэродинамической модели к интерфейсу IAeroModel
 *
 * Не содержит собственной математики — только делегирует расчёты
 * в существующий класс FullAeroModel, чтобы не нарушать инварианты
 * и сохранить воспроизводимость результатов.
 */
template<typename metricType>
class FullAeroModelAdapter final : public IAeroModel<metricType> {
public:
    FullAeroModelAdapter(
        const RocketAeroInput<metricType>& aero_input,
        std::shared_ptr<ComponentInterpolationManager<metricType>> interp_mgr)
        : full_model_(aero_input, interp_mgr),
          center_of_mass_(static_cast<metricType>(0))
    {
        if (!interp_mgr) {
            throw std::invalid_argument("FullAeroModelAdapter: interpolation manager cannot be null");
        }
    }

    ~FullAeroModelAdapter() override = default;

    Eigen::Vector3<metricType> computeAerodynamicForces(
        const Eigen::Vector3<metricType>& velocity_body,
        metricType air_density,
        metricType /*mach_number*/) const override
    {
        // Вся математика — внутри FullAeroModel
        return full_model_.computeAerodynamicForces(velocity_body, air_density);
    }

    Eigen::Vector3<metricType> computeAerodynamicMoments(
        const Eigen::Vector3<metricType>& velocity_body,
        const Eigen::Vector3<metricType>& angular_velocity,
        metricType air_density,
        metricType /*mach_number*/,
        const Eigen::Vector3<metricType>& /*euler_angles*/) const override
    {
        // Моменты тоже полностью считаются в FullAeroModel
        return full_model_.computeAerodynamicMoments(
            velocity_body,
            angular_velocity,
            air_density,
            center_of_mass_,
            current_deflections_);
    }

    /**
     * \brief Обновление текущих управляющих воздействий
     *
     * Вызывается автопилотом или внешней логикой управления.
     * Здесь нет физики — только передача параметров в полную модель.
     */
    void updateControl(
        metricType center_of_mass,
        const std::vector<metricType>& deflections)
    {
        center_of_mass_ = center_of_mass;
        full_model_.updateControlSurfaceDeflections(deflections);
        current_deflections_ = deflections;
    }

private:
    FullAeroModel<metricType> full_model_;
    metricType center_of_mass_;
    std::vector<metricType> current_deflections_;
};

