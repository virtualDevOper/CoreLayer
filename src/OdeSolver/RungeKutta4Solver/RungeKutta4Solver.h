/*//
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
class RungeKutta4Solver final : public ODESolver<metricType,CallbackType> {
public:
    void solve(
        std::shared_ptr<ObjectManager<metricType>> object_manager,
        metricType t_start,
        metricType step_size,
        CallbackType continue_callback,
        std::shared_ptr<SimulationMomento<metricType>> momento
        ) override {

        if (step_size <= 0) {throw std::invalid_argument("Шаг по времени должен быть положительным, сейчас он: " + std::to_string(step_size));}
        if (t_start < 0) {throw std::invalid_argument("Начальное время не может быть отрицательным, сейчас оно : " + std::to_string(t_start));}
        if (object_manager.getObjectCount() == 0) {throw std::invalid_argument("Нет объектов для обработки!");}
        metricType current_time = t_start;
        momento.saveStartParams(object_manager.getAllObjects());
        const auto& all_objects = object_manager.getAllObjects();

        while (true) {
            for (const auto& id_obj : all_objects) {  // работаем с const ссылками
                const auto& id = id_obj.first;
                const auto& object = id_obj.second;   // ссылка на объект

                const auto& current_stateStorage = momento.getStateStorageByID(id);
                const auto& current_state = current_stateStorage->getStates().back();

                if (object->isActive()) {
                    // Предполагая, что solveOneStepForOneObj возвращает по значению
                    auto new_state = solveOneStepForOneObj(
                        current_state, current_time, step_size, object->getDynamicSys());
                    momento.addSnapshotByID(id, std::move(new_state));  // перемещаем вместо копирования
                } else {
                    momento.addSnapshotByID(id, current_state);  // здесь копирование неизбежно
                }
            }

            current_time += step_size;

            // Получаем ID столкнувшихся объектов
            auto collided_objects_IDs = this->checkCollisions(momento.viewTrackedObjs());
            freezeCollidedObjects(object_manager, std::move(collided_objects_IDs));  // перемещаем

            if (!shouldContinue(continue_callback, current_time, momento)) {
                break;
            }
        }
}

private:
    static bool shouldContinue(CallbackType& continue_callback, metricType current_time,
                               SimulationMomento<metricType>& momento) {
        return continue_callback(current_time, momento);
    }

    void freezeCollidedObjects(ObjectManager<metricType>& object_manager,
                          const std::vector<int>& collided_objects_IDs) {
        for (int collided_id : collided_objects_IDs) {
            auto object = object_manager.getObjectByID(collided_id);
            if (object != nullptr && object->isActive()) {
                object->setCollided();
                object->setVelocity(Eigen::Matrix<metricType, 3, 1>::Zero());
                object->setAcceleration(Eigen::Matrix<metricType, 3, 1>::Zero());
                object->setTotalForse(Eigen::Matrix<metricType, 3, 1>::Zero());
                object->setTotalMoment(Eigen::Matrix<metricType, 3, 1>::Zero());
                object->setAngularAcceleration(Eigen::Matrix<metricType, 3, 1>::Zero());
                object->setAngularVelocity(Eigen::Matrix<metricType, 3, 1>::Zero());
            }
        }
    }

    ObjSnapshot<metricType> solveOneStepForOneObj(
        const ObjSnapshot<metricType>& current_state,
        metricType current_time,
        metricType step_size,
        IDynamicsSystem<metricType>& sys
        )
    {
        auto deriv_func = [&](const ObjSnapshot<metricType>& state, metricType time) {
            return sys.get_rhs_derivatives(state, time);
        };
        auto k1 = deriv_func(current_state, current_time);
        auto state_k2 = current_state + (step_size * 0.5) * k1;
        auto k2 = deriv_func(state_k2, current_time + step_size * 0.5);
        auto state_k3 = current_state + (step_size * 0.5) * k2;
        auto k3 = deriv_func(state_k3, current_time + step_size * 0.5);
        auto state_k4 = current_state + step_size * k3;
        auto k4 = deriv_func(state_k4, current_time + step_size);
        return current_state + (step_size / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }
};*/