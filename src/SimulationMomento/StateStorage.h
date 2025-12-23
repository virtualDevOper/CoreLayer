
//
// Created by 4NR_Operator_3 on 06.10.2025.
//

#pragma once
#pragma once
#include "../utils/ObjSnapshot.h"

/**
 * \brief Класс для хранения снимков состояний определенного объекта
 *
 * \tparam metricType Тип данных для метрических величин
 *
* \details  Реализует специализацию паттерна "Хранитель" для захвата и сохранения
 * последовательных снимков состояния определенного ЛА
*/





template <typename MetricType>
class StateStorage {
public:
    StateStorage(const StateStorage&) = delete;
    StateStorage& operator=(const StateStorage&) = delete;

    explicit StateStorage(int id) : id_(id) {}

    StateStorage(StateStorage&& other) noexcept
        : id_(other.id_), states_(std::move(other.states_)) {}

    // Две версии addState для разных случаев
    void addState(const ObjSnapshot<MetricType>& snapshot) {
        states_.push_back(snapshot);
    }

    void addState(ObjSnapshot<MetricType>&& snapshot) {
        states_.push_back(std::move(snapshot));
    }

    void addState(std::unique_ptr<ObjSnapshot<MetricType>> snapshot) {
        states_.push_back(std::move(*snapshot));
    }

    const std::vector<ObjSnapshot<MetricType>>& getStates() const { return states_; }
    [[nodiscard]] int getId() const { return id_; }

private:
    int id_;
    std::vector<ObjSnapshot<MetricType>> states_;
};

// // Включаем реализацию шаблона
// #include "StateStorage.tpp"


/*
/#2#/ В коде SimulationMomento:
auto momento = std::make_unique<SimulationMomento<double>>();
// Устанавливаем стратегию сохранения в CSV
momento->setStrategy(std::make_unique<CsvSaveStrategy<double>>("simulation_data.csv"));
// Добавляем отслеживаемые объекты
std::vector<StateStorage<double>> trackedObjects;
// Создаем объект 1
StateStorage<double> obj1(1);
obj1.addState(ObjSnapshot<double>(1, {{"velocity", 250.5}, {"altitude", 10000.0}}));
obj1.addState(ObjSnapshot<double>(1, {{"velocity", 248.7}, {"altitude", 9950.0}}));
// Создаем объект 2
StateStorage<double> obj2(2);
obj2.addState(ObjSnapshot<double>(2, {{"temperature", -45.2}, {"pressure", 1013.2}}));
trackedObjects.push_back(std::move(obj1));
trackedObjects.push_back(std::move(obj2));
momento->addTrackedObjs(std::move(trackedObjects));
// Сохраняем в CSV
momento->save();#2#
#1#
*/


