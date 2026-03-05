#pragma once
#include "PCH.h"
#include "DynamicParametersProvider.h"
#include "IParameterProvider.h"
#include "../Interpolation/ComponentInterpolationManager.h"

template<typename metricType>
class ParameterProviderFactory {
public:
    /**
     * \brief Создаёт DynamicParametersProvider из менеджера интерполяции
     * \param manager Менеджер интерполяции (shared_ptr)
     * \param need_mass Включить провайдер массы
     * \param need_inertia Включить провайдер инерции
     * \param need_thrust Включить провайдер тяги
     * \param need_com Включить провайдер центра масс
     * \return Настроенный DynamicParametersProvider
     */
    [[nodiscard]] static DynamicParametersProvider<metricType> createFromManager(
        std::shared_ptr<ComponentInterpolationManager<metricType>> manager,
        bool need_mass = true,
        bool need_inertia = true,
        bool need_thrust = true,
        bool need_com = true) noexcept
    {
        std::shared_ptr<IMassProvider<metricType>> mass_ptr = nullptr;
        std::shared_ptr<IInertiaProvider<metricType>> inertia_ptr = nullptr;
        std::shared_ptr<IThrustProvider<metricType>> thrust_ptr = nullptr;
        std::shared_ptr<ICOMProvider<metricType>> com_ptr = nullptr;

        if (manager) {
            if (need_mass) {
                mass_ptr = std::static_pointer_cast<IMassProvider<metricType>>(manager);
            }
            if (need_inertia) {
                inertia_ptr = std::static_pointer_cast<IInertiaProvider<metricType>>(manager);
            }
            if (need_thrust) {
                thrust_ptr = std::static_pointer_cast<IThrustProvider<metricType>>(manager);
            }
            if (need_com) {
                com_ptr = std::static_pointer_cast<ICOMProvider<metricType>>(manager);
            }
        }

        return DynamicParametersProvider<metricType>(
            std::move(mass_ptr),
            std::move(inertia_ptr),
            std::move(thrust_ptr),
            std::move(com_ptr)
        );
    }

    /**
     * \brief Создаёт провайдер для "точечной" модели (только масса и COM)
     * \param manager Менеджер интерполяции
     * \return DynamicParametersProvider без инерции и тяги
     */
    [[nodiscard]] static DynamicParametersProvider<metricType> createPointMass(
        std::shared_ptr<ComponentInterpolationManager<metricType>> manager) noexcept
    {
        return createFromManager(manager, true, false, false, true);
    }

    /**
     * \brief Создаёт провайдер для аэродинамической модели (без массы/инерции/тяги)
     * \param manager Менеджер интерполяции
     * \return DynamicParametersProvider только с COM
     */
    [[nodiscard]] static DynamicParametersProvider<metricType> createAeroOnly(
        std::shared_ptr<ComponentInterpolationManager<metricType>> manager) noexcept
    {
        return createFromManager(manager, false, false, false, true);
    }

    /**
     * \brief Создаёт провайдер для полной модели ракеты (все параметры)
     * \param manager Менеджер интерполяции
     * \return DynamicParametersProvider со всеми параметрами
     */
    [[nodiscard]] static DynamicParametersProvider<metricType> createFullRocket(
        std::shared_ptr<ComponentInterpolationManager<metricType>> manager) noexcept
    {
        return createFromManager(manager, true, true, true, true);
    }

    /**
     * \brief Создаёт провайдер для баллистической модели (масса + COM, без тяги/инерции)
     * \param manager Менеджер интерполяции
     * \return DynamicParametersProvider для баллистики
     */
    [[nodiscard]] static DynamicParametersProvider<metricType> createBallistic(
        std::shared_ptr<ComponentInterpolationManager<metricType>> manager) noexcept
    {
        return createFromManager(manager, true, false, false, true);
    }
};