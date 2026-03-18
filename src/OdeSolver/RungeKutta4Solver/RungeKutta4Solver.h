//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "PCH.h"
#include "../ODESolver.h"
#include "../../utils/KinematicStateDerivative/KinematicStateDerivative.h"
#include "logging/Logger.h"

/**
 * \brief  Метод Рунге–Кутты 4 для интегрирования систем ОДУ.
 *
 * \tparam metricType  Тип данных для метрических величин.
 * \tparam CallbackType Тип колбэка продолжения интегрирования (останавливает расчёт по условию).
 *
 * \details Интегрирует только кинематическую часть состояния (KinematicState),
 * вызывая IDynamicsSystem::get_rhs_derivatives. На каждом шаге сохраняет
 * снимки состояний в SimulationMomento и использует колбэк для проверки
 * условий остановки (например, удар о землю).
 */
template <typename metricType, typename CallbackType>
class RungeKutta4Solver final : public ODESolver<metricType, CallbackType> {
public:
    RungeKutta4Solver() = default;
    
    void solve(
        std::shared_ptr<IObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        SimulationMomento<metricType>& momento
    ) override;

private:
    std::unique_ptr<ObjSnapshot<metricType>> solveOneStepForOneObj(
        const ObjSnapshot<metricType>& current_snapshot,
        metricType current_time,
        metricType step_size,
        IDynamicsSystem<metricType>* sys
    );
};


template <typename metricType, typename CallbackType>
void RungeKutta4Solver<metricType, CallbackType>::solve(
    std::shared_ptr<IObjectManager<metricType>> object_manager,
    metricType t_start,
    metricType step_size,
    CallbackType continue_callback,
    SimulationMomento<metricType>& momento
) {
    if (step_size <= 0) {
        throw std::invalid_argument("Шаг по времени должен быть положительным: " + std::to_string(step_size));
    }
    if (t_start < 0) {
        throw std::invalid_argument("Время начала расчета не может быть негативным: " + std::to_string(t_start));
    }
    if (!object_manager || object_manager->getObjectCount() == 0) {
        throw std::invalid_argument("Нет объектов для расчетов");
    }
    if (!continue_callback) {
        throw std::invalid_argument("Continue callback не установлен");
    }

    metricType current_time = t_start;
    momento.saveStartParams(object_manager->getAllObjects());
    const auto& all_objects = object_manager->getAllObjects();

    while (true) {
        for (const auto& [id, weak_object] : all_objects) {
            auto object = weak_object.lock();
            if (!object) continue;

            auto& current_state_storage = momento.getStateStorageByID(id);
            const auto& states = current_state_storage.getStates();
            if (states.empty()) {
                throw std::runtime_error("Object storage: state is empty: " + std::to_string(id));
            }
            const auto& current_state = states.back();

            if (object->isActive()) {
                try {
                    auto new_state = solveOneStepForOneObj(
                        current_state, current_time, step_size, object->getDynamicSys()
                    );
                    // Создаём копию для momento перед перемещением
                    //по идее больше тут ничего не нужно, это максиму, однако можно и кватернионы добавить и тд, так что можешь дописать
                    auto new_state_copy = ObjSnapshot<metricType>::createBuilder(new_state->getKinematics())
                        .setTime(new_state->getTime())
                        .setMass(new_state->getMass())
                        .setInertia(new_state->getInertia())
                        .setTotalForce(new_state->getTotalForce())
                        .setTotalMoment(new_state->getTotalMoment())
                        .setAeroCx(static_cast<metricType>(new_state->getAeroCx()))
                        .setAeroCy(static_cast<metricType>(new_state->getAeroCy()))
                        .setAeroCz(static_cast<metricType>(new_state->getAeroCz()))
                        .setAeroMx(static_cast<metricType>(new_state->getAeroMx()))
                        .setAeroMy(static_cast<metricType>(new_state->getAeroMy()))
                        .setAeroMz(static_cast<metricType>(new_state->getAeroMz()))
                        .setAeroAlpha(new_state->getAeroAlpha())
                        .setAeroBeta(new_state->getAeroBeta())
                        .setAeroMach(new_state->getAeroMach())
                        .setAeroXcp(static_cast<metricType>(new_state->getAeroXcp()))
                        .setAeroStaticMargin(static_cast<metricType>(new_state->getAeroStaticMargin()))
                        .buildUnique();
                    
                    // Обновляем состояние объекта для колбэков
                    object->updateSnapshot(std::move(new_state));
                    // Сохраняем копию в momento
                    momento.addSnapshotByID(id, std::move(new_state_copy));
                } catch (const std::exception& e) {
                    Logger::getInstance().error("Derivatives calculation error for obj ID: " + std::to_string(id) + " in time: " + std::to_string(current_time) + ": " + std::string(e.what()));
                    throw;
                }
            } else {
                momento.addSnapshotByID(id, current_state);
            }
        }

        current_time += step_size;
        
        try {
            if (!continue_callback(object_manager, current_time)) {
                break;
            }
        } catch (const std::exception& e) {
            Logger::getInstance().error("Continue callback err: " + std::string(e.what()) );
            break;
        }
    }
}

template <typename metricType, typename CallbackType>
std::unique_ptr<ObjSnapshot<metricType>> RungeKutta4Solver<metricType, CallbackType>::solveOneStepForOneObj(
    const ObjSnapshot<metricType>& current_snapshot,
    metricType current_time,
    metricType step_size,
    IDynamicsSystem<metricType>* sys
) {
    const auto& current_kinematics = current_snapshot.getKinematics();

    // Standard RK4 implementation
    auto k1 = *sys->get_rhs_derivatives(current_kinematics, current_time);
    
    metricType half_step = step_size * metricType(0.5);
    auto k2 = *sys->get_rhs_derivatives(current_kinematics + half_step * k1, current_time + half_step);
    auto k3 = *sys->get_rhs_derivatives(current_kinematics + half_step * k2, current_time + half_step);
    auto k4 = *sys->get_rhs_derivatives(current_kinematics + step_size * k3, current_time + step_size);

    auto new_kinematics = current_kinematics + (step_size / metricType(6.0)) * (k1 + metricType(2.0) * k2 + metricType(2.0) * k3 + k4);

    return sys->augmentSnapshot(new_kinematics, current_time + step_size);
}
