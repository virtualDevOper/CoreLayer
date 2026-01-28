//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../AbstractObject.h"

/**
 * \brief Абстрактный класс летательного аппарата с аэродинамическими параметрами.
 *
 * \tparam metricType Тип данных для метрических величин.
 * \tparam AeroInput  Тип структуры, описывающей аэродинамический вход (геометрия, рулевые поверхности и т.п.).
 *
 * \details Расширяет AbstractObject хранилищем аэродинамического ввода и
 * менеджером интерполяции параметров (масса, инерция, тяга и др.).
 * Конкретные ЛА (ракеты, снаряды, БПЛА) наследуются от этого класса.
 */
template <typename metricType, typename AeroInput>
class AbstractAircraft : public AbstractObject<metricType> {
public:
    explicit AbstractAircraft(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<AeroInput> aero_input,
        std::shared_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr)
        : AbstractObject<metricType>(std::move(sys), std::move(initial_params)),
          aero_input_(std::move(aero_input)),
          comp_interp_mgr_(comp_interp_mgr)
    {
        if (!aero_input_) {
            throw std::invalid_argument("AeroInput cannot be null");
        }

        if (!comp_interp_mgr_) {
            throw std::invalid_argument("ComponentInterpolationManager cannot be null");
        }
    }

protected:
    std::unique_ptr<AeroInput> aero_input_;
    std::shared_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr_;
};