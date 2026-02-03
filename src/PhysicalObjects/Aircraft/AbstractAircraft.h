//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../AbstractObject.h"
#include "../../utils/IParameterProvider.h"

/**
 * \brief Абстрактный класс летательного аппарата с аэродинамическими параметрами.
 *
 * \tparam metricType Тип данных для метрических величин.
 * \tparam AeroInput  Тип структуры, описывающей аэродинамический вход (геометрия, рулевые поверхности и т.п.).
 *
 * \details Расширяет AbstractObject хранилищем аэродинамического ввода и
 * слабой ссылкой на провайдер параметров (для разрыва циклических зависимостей).
 * Конкретные ЛА (ракеты, снаряды, БПЛА) наследуются от этого класса.
 */
template <typename metricType, typename AeroInput>
class AbstractAircraft : public AbstractObject<metricType> {
public:
    explicit AbstractAircraft(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<AeroInput> aero_input,
        std::weak_ptr<IParameterProvider<metricType>> param_provider)  // Изменено: weak_ptr для наблюдения
        : AbstractObject<metricType>(std::move(sys), std::move(initial_params)),
          aero_input_(std::move(aero_input)),
          param_provider_(param_provider)
    {
        if (!aero_input_) {
            throw std::invalid_argument("AeroInput cannot be null");
        }

        // Проверяем, что провайдер параметров доступен
        if (param_provider_.expired()) {
            throw std::invalid_argument("ParameterProvider is not available");
        }
    }

protected:
    std::unique_ptr<AeroInput> aero_input_;
    std::weak_ptr<IParameterProvider<metricType>> param_provider_;  // Наблюдаем, не владеем

    // Защищенный метод для получения провайдера параметров
    [[nodiscard]] std::shared_ptr<IParameterProvider<metricType>> getParameterProvider() const {
        auto provider = param_provider_.lock();
        if (!provider) {
            throw std::runtime_error("ParameterProvider is no longer available");
        }
        return provider;
    }
};