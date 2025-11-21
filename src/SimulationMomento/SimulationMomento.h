/*
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
 #1#

//паттерн Memento.

template <typename metricType>
class SimulationMomento {
public:
    SimulationMomento() = default;
    ~SimulationMomento() = default;
    void addTrackedObjs( std::vector<StateStorage<metricType>>&& trackedObjs) {trackedStates_ = std::move(trackedObjs);}
    void setStrategy(std::shared_ptr<SaveStrategy<metricType>> strategy) {strategy_ = strategy;}
    void save() {strategy_->save(trackedStates_);}
    const std::vector<StateStorage<metricType>>& viewTrackedObjs() const {return trackedStates_;}
    void addSnapshotByID(int id, std::shared_ptr<ObjSnapshot<metricType>> snapshot) {
        for (auto& tracked_state : trackedStates_) {
            if (tracked_state.getID() == id) {
                tracked_state.addState(std::move(snapshot));
                return;
            }
        }
        throw std::runtime_error("Объект с id " + std::to_string(id) + " не найден");
    }
    void saveStartParams(std::vector<std::pair<int, std::shared_ptr<AbstractObject<metricType>>>> objects) {
        for (const auto& [id, obj] : objects) {
            auto objectStateSnapshot = obj->getStateSnapshot();
            auto newState = std::make_shared<StateStorage<metricType>>();
            newState->addState(objectStateSnapshot);
            trackedStates_.push_back(newState);
        }
    }
    StateStorage<metricType>& getStateStorageByID(int id) {
        for (auto& tracked_state : trackedStates_) {
            if (tracked_state.getID() == id) {
                return tracked_state;
            }
        }
        throw std::runtime_error("StateStorage with ID " + std::to_string(id) + " not found");
    }
private:
    std::vector<StateStorage<metricType>> trackedStates_;
    std::shared_ptr<SaveStrategy<metricType>> strategy_;
    // для варианта хранения в оперативке создается приватное поле данных
    // для варианта хранения в бд отправляется запись в бд(вызывается деструктор)(либо после каждого фрейма, либо для каждого элемента)
    // для варианта записи в текстовый файл(вызывается деструктор), файл не закрывается и просто по мере работы записывается в него
};
*/

