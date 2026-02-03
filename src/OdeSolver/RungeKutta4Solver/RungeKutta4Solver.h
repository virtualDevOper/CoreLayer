//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "PCH.h"
#include "../ODESolver.h"
#include "../../utils/KinematicState.h"  // ← НОВАЯ ЗАВИСИМОСТЬ

/**
 * \brief Четвёртый порядок метода Рунге–Кутты для интегрирования систем ОДУ.
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
    RungeKutta4Solver() = default;  // Конструктор по умолчанию
    
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
        IDynamicsSystem<metricType>* sys  // Изменено: принимаем сырой указатель для наблюдения
    );
};


// === РЕАЛИЗАЦИЯ ШАБЛОНА (должна быть в заголовочном файле) ===

template <typename metricType, typename CallbackType>
void RungeKutta4Solver<metricType, CallbackType>::solve(
    std::shared_ptr<IObjectManager<metricType>> object_manager,
    metricType t_start,
    metricType step_size,
    CallbackType continue_callback,
    SimulationMomento<metricType>& momento
) {
    if (step_size <= 0) {
        throw std::invalid_argument("Step size must be positive: " + std::to_string(step_size));
    }
    if (t_start < 0) {
        throw std::invalid_argument("Start time cannot be negative: " + std::to_string(t_start));
    }
    if (!object_manager || object_manager->getObjectCount() == 0) {
        throw std::invalid_argument("No objects to process");
    }
    if (!continue_callback) {
        throw std::invalid_argument("Continue callback not set");
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
                throw std::runtime_error("State storage is empty for object ID: " + std::to_string(id));
            }
            const auto& current_state = states.back();

            if (object->isActive()) {
                try {
                    auto new_state = solveOneStepForOneObj(
                        current_state, current_time, step_size, object->getDynamicSys()
                    );
                    momento.addSnapshotByID(id, std::move(new_state));
                } catch (const std::exception& e) {
                    std::cerr << "Error computing derivatives for object ID " << id
                              << " at time " << current_time << ": " << e.what() << std::endl;
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
            std::cerr << "Error in continue callback: " << e.what() << std::endl;
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
