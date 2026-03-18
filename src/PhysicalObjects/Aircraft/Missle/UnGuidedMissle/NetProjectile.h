#pragma once
#include "PhysicalObjects/AbstractObject.h"
#include "utils/Interpolation/ComponentInterpolationManager.h"

/**
 * @brief Негидируемый сетевой снаряд, использующий 3DOF-модель.
 */
template<typename metricType>
class NetProjectile final : public AbstractObject<metricType> {
private:
    std::shared_ptr<ComponentInterpolationManager<metricType>> interp_manager_;

public:
    explicit NetProjectile(
        std::unique_ptr<IDynamicsSystem<metricType>>   sys,
        std::unique_ptr<ObjInitParams<metricType>>     initial_params,
        std::shared_ptr<ComponentInterpolationManager<metricType>> interp_mgr
    )
        : AbstractObject<metricType>(std::move(sys), std::move(initial_params))
        , interp_manager_(std::move(interp_mgr))
    { }

    [[nodiscard]] std::shared_ptr<ComponentInterpolationManager<metricType>>
    getInterpolationManager() const {
        return interp_manager_;
    }
};