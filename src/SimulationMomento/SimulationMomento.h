//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../../PCH.h"
#include "StateStorage.h"
#include "SaveStrategy/SaveStrategy.h"
#include "../utils/ObjSnapshot.h"
#include "../PhysicalObjects/AbstractObject.h"




/**
 * \brief менеджер состояний всей симуляции
 *
 * \tparam metricType Тип данных для метрических величин
 *
* \details
* Тут мы создаем копию переданных состояний, чтобы не было взаимодействия лишнего, этот класс ничего не должен знать
* об объектах, хранить только их состояния.
 */

//паттерн Memento.


template <typename metricType>
class SimulationMomento {
public:
    SimulationMomento() = default;
    ~SimulationMomento() = default;

    void addTrackedObjs(std::vector<StateStorage<metricType>>&& trackedObjs) {
        trackedStates_ = std::move(trackedObjs);
    }

    void setStrategy(std::shared_ptr<SaveStrategy<metricType>> strategy) {
        strategy_ = std::move(strategy);
    }

    void save() {
        if (!strategy_) {
            throw std::runtime_error("Save strategy not set");
        }
        strategy_->save(trackedStates_);
    }

    const std::vector<StateStorage<metricType>>& viewTrackedObjs() const {return trackedStates_;}

    // Три версии для разных случаев
    void addSnapshotByID(const int id, const ObjSnapshot<metricType>& snapshot) {getStateStorageByID(id).addState(snapshot);}
    void addSnapshotByID(const int id, ObjSnapshot<metricType>&& snapshot) {getStateStorageByID(id).addState(std::move(snapshot));}
    void addSnapshotByID(const int id, std::unique_ptr<ObjSnapshot<metricType>> snapshot) {getStateStorageByID(id).addState(std::move(snapshot));}

    void saveStartParams(const std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>>& objects) {
        for (const auto& [id, weak_obj] : objects) {
            auto obj = weak_obj.lock(); // Получаем shared_ptr из weak_ptr
            if (!obj) continue; // Пропускаем уничтоженные объекты
            auto objectStateSnapshot = obj->getStateSnapshot();
            StateStorage<metricType> newState(id);
            newState.addState(objectStateSnapshot);
            trackedStates_.push_back(std::move(newState));
        }
    }

    StateStorage<metricType>& getStateStorageByID(int id) {
        for (auto& tracked_state : trackedStates_) {
            if (tracked_state.getId() == id) {
                return tracked_state;
            }
        }
        throw std::runtime_error("StateStorage с ID " + std::to_string(id) + " не найдена");
    }

private:
    std::vector<StateStorage<metricType>> trackedStates_;
    std::shared_ptr<SaveStrategy<metricType>> strategy_;
};
