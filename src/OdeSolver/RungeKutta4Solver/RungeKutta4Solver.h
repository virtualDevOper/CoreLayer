//
// Created by 4NR_Operator_3 on 16.09.2025.
//
#pragma once
#include "../../../PCH.h"
#include "../ODESolver.h"

//TODO
// === Придумать обработку объектов с состоянием DESTROYED,COLLIDED  ===
// === В теории можно добавить количество состояний и изменить логику вычислений по ним  ===
// === Исправь типи переделанные



template <typename metricType, typename CallbackType>
class RungeKutta4Solver final : public ODESolver<metricType, CallbackType> {
public:
    void solve(
        std::shared_ptr<ObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        SimulationMomento<metricType>& momento
    ) override {

        if (step_size <= static_cast<metricType>(0)) {throw std::invalid_argument("Шаг по времени должен быть положительным, сейчас он: " + std::to_string(step_size));}
        if (t_start < static_cast<metricType>(0)) {throw std::invalid_argument("Начальное время не может быть отрицательным, сейчас оно: " + std::to_string(t_start));}
        if (!object_manager || object_manager->getObjectCount() == 0) {throw std::invalid_argument("Нет объектов для обработки!");}

        metricType current_time = t_start;
        momento.saveStartParams(object_manager->getAllObjects());
        const auto& all_objects = object_manager->getAllObjects();

        while (true) {
            for (const auto& [id, object] : all_objects) {
                if (!object) continue;

                auto& current_state_storage = momento.getStateStorageByID(id);
                const auto& current_state = current_state_storage.getStates().back();

                if (object->isActive()) {
                    auto new_state = solveOneStepForOneObj(
                        current_state, current_time, step_size, object->getDynamicSys()
                    );
                    momento.addSnapshotByID(id, std::move(new_state));
                } else { // если неактивен, то замораживается
                    momento.addSnapshotByID(id, current_state);
                }
            }
            current_time += step_size;
            auto collided_objects_IDs = this->checkCollisions(momento.viewTrackedObjs());
            freezeCollidedObjects(object_manager, collided_objects_IDs);

            if (!continue_callback(object_manager)) {
                break;
            }
        }
    }

private:
    void freezeCollidedObjects(const std::shared_ptr<ObjectManager<metricType>>& object_manager,
                               const std::vector<int>& collided_objects_IDs) {
        for (int collided_id : collided_objects_IDs) {
            auto object = object_manager->getObjectByID(collided_id);
            if (object && object->isActive()) {
                object->setCollided();
                // Убрать вызовы несуществующих методов
                // Просто устанавливаем состояние COLLIDED
            }
        }
    }



//по идее тут остановился но можешь

    std::unique_ptr<ObjSnapshot<metricType>> solveOneStepForOneObj(
        const ObjSnapshot<metricType>& current_state,
        metricType current_time,
        metricType step_size,
        std::shared_ptr<IDynamicsSystem<metricType>> sys
    ) {
        auto deriv_func = [&](const ObjSnapshot<metricType>& state, metricType time)
            -> std::unique_ptr<ObjSnapshot<metricType>> {
            return sys->get_rhs_derivatives(state, time);
        };

        // Получаем производные как unique_ptr
        auto k1_ptr = deriv_func(current_state, current_time);
        // Разыменовываем для использования в операциях
        const auto& k1 = *k1_ptr;

        // Используем явное приведение типов
        metricType half_step = step_size * static_cast<metricType>(0.5);
        auto state_k2 = current_state + half_step * k1;

        auto k2_ptr = deriv_func(state_k2, current_time + half_step);
        const auto& k2 = *k2_ptr;
        auto state_k3 = current_state + half_step * k2;

        auto k3_ptr = deriv_func(state_k3, current_time + half_step);
        const auto& k3 = *k3_ptr;
        auto state_k4 = current_state + step_size * k3;

        auto k4_ptr = deriv_func(state_k4, current_time + step_size);
        const auto& k4 = *k4_ptr;

        // Вычисляем новое состояние
        metricType one_sixth = static_cast<metricType>(1.0) / static_cast<metricType>(6.0);
        auto result_state = current_state +
            (step_size * one_sixth) * (k1 + static_cast<metricType>(2.0) * k2 +
                                       static_cast<metricType>(2.0) * k3 + k4);

        return std::make_unique<ObjSnapshot<metricType>>(std::move(result_state));
    }
};
