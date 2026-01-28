//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "../../../PCH.h"
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
    void solve(
        std::shared_ptr<IObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        SimulationMomento<metricType>& momento
    ) override {
        if (step_size <= static_cast<metricType>(0)) {
            throw std::invalid_argument("Шаг по времени должен быть положительным, сейчас он: " + std::to_string(step_size));
        }
        if (t_start < static_cast<metricType>(0)) {
            throw std::invalid_argument("Начальное время не может быть отрицательным, сейчас оно: " + std::to_string(t_start));
        }
        if (!object_manager || object_manager->getObjectCount() == 0) {
            throw std::invalid_argument("Нет объектов для обработки!");
        }
        if (!continue_callback) {
            throw std::invalid_argument("Callback для продолжения симуляции не установлен!");
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
            // Временно удаляем коллизии (пока не реализованы)
            // auto collided_objects_IDs = this->checkCollisions(momento.viewTrackedObjs());
            // freezeCollidedObjects(object_manager, collided_objects_IDs);

            try {
                if (!continue_callback(object_manager)) {
                    break;
                }
                if (current_time > 2) {
                    break;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error in continue callback: " << e.what() << std::endl;
                break;
            }
        }
    }

private:
    std::unique_ptr<ObjSnapshot<metricType>> solveOneStepForOneObj(
        const ObjSnapshot<metricType>& current_snapshot,
        metricType current_time,
        metricType step_size,
        std::shared_ptr<IDynamicsSystem<metricType>> sys
    ) {
        const auto& current_kinematics = current_snapshot.getKinematics();

        auto deriv_func = [&](const KinematicState<metricType>& state, metricType time)
            -> std::unique_ptr<KinematicState<metricType>> {
            return sys->get_rhs_derivatives(state, time);
        };

        // РК4 работает ТОЛЬКО с кинематикой
        auto k1 = *deriv_func(current_kinematics, current_time);
        metricType half_step = step_size * static_cast<metricType>(0.5);
        auto k2 = *deriv_func(current_kinematics + half_step * k1, current_time + half_step);
        auto k3 = *deriv_func(current_kinematics + half_step * k2, current_time + half_step);
        auto k4 = *deriv_func(current_kinematics + step_size * k3, current_time + step_size);

        metricType one_sixth = static_cast<metricType>(1.0) / static_cast<metricType>(6.0);
        auto new_kinematics = current_kinematics + (step_size * one_sixth) * (k1 +
            static_cast<metricType>(2.0) * k2 +
            static_cast<metricType>(2.0) * k3 +
            k4);

        // Добавляем параметры через augmentSnapshot()
        return sys->augmentSnapshot(new_kinematics, current_time + step_size);
    }
};