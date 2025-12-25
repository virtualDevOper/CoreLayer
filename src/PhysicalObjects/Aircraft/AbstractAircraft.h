//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../AbstractObject.h"


/**
 * \brief Абстрактный класс летательного аппарата
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Под это описание подходит любая ракета/снаряд/БПЛА
 * Аэродинамику в любом случае здесь прийдется учитывать
 * однако не во всех аппаратах происходит управление
 */

template <typename metricType, typename AeroInput>
class AbstractAircraft : public AbstractObject<metricType> {
public:
    explicit AbstractAircraft(
        std::unique_ptr<IDynamicsSystem<metricType>> sys,
        std::unique_ptr<ObjInitParams<metricType>> initial_params,
        std::unique_ptr<AeroInput> aero_input,
        std::unique_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr)
        : AbstractObject<metricType>(std::move(sys), std::move(initial_params)),
          aero_input_(std::move(aero_input)),
          comp_interp_mgr_(std::move(comp_interp_mgr)) {

        if (!comp_interp_mgr_) {
            throw std::invalid_argument("ComponentInterpolationManager не может быть null");
        }
    }

    virtual ~AbstractAircraft() = default;

    // === ЧИСТЫЕ ВИРТУАЛЬНЫЕ МЕТОДЫ ДЛЯ НАСЛЕДНИКОВ ===
    virtual Eigen::Vector3<metricType> getAerodynamicForces(metricType t) const = 0;
    virtual Eigen::Vector3<metricType> getAerodynamicMoments(metricType t) const = 0;

    // === ДОСТУП К ПАРАМЕТРАМ (чтобы было меньше шансов получить исключение из компонент интерполятора) ===
    metricType getMass(metricType t) const {return comp_interp_mgr_->getMass(t);}

    Eigen::Vector3<metricType> getInertia(metricType t) const {return comp_interp_mgr_->getInertia(t);}

    Eigen::Vector3<metricType> getThrust(metricType t) const {return comp_interp_mgr_->getThrust(t);}

    const ComponentInterpolationManager<metricType>* getComponentInterpolation() const {return comp_interp_mgr_.get();}

protected:
    std::unique_ptr<AeroInput> aero_input_;
    std::unique_ptr<ComponentInterpolationManager<metricType>> comp_interp_mgr_;
};