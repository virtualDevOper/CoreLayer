//
// Created by 4NR_Operator_3 on 28.10.2025.
//

#pragma once
#include "AbstractObject.h"
#include "../../PCH.h"
#include "Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h"


/**
 * \brief Класс для отслеживания динамических объектов
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Этот класс не хранит параметры динамических объектов, такие как координаты
 * этот класс отслеживает жизненный цикл объектов и имеет ссылки на них(а вот то, что внутри абстрактного ЛА - уже в ЛА смотри).
 * Скорее всего в ЛА будет решаться задача динамики полета с датчиков, так что смотри по ситуации.
 */
template <typename metricType>
class ObjectManager {
private:
    std::unordered_map<int, std::shared_ptr<AbstractObject<metricType>>> all_objects_;
    std::atomic<int> next_id_{0};
    mutable std::mutex mutex_; // mutable позволяет изменение члена класса даже в константных функциях-членах(пенисах)


public:
    ObjectManager() = default;
    ObjectManager(const ObjectManager&) = delete;
    ObjectManager& operator=(const ObjectManager&) = delete;


    int addTrackedObject(std::shared_ptr<AbstractObject<metricType>> object) {
        std::lock_guard<std::mutex> lock(mutex_);
        int id = next_id_++;
        all_objects_.emplace(id, std::move(object));
        return id;
    }

    std::shared_ptr<AbstractObject<metricType>> getObjectByID(int id) const {
        std::lock_guard lock(mutex_);
        auto it = all_objects_.find(id);
        return (it != all_objects_.end()) ? it->second : nullptr;
    }

    bool removeObjectById(int id) {
        std::lock_guard lock(mutex_);
        return all_objects_.erase(id) > 0;
    }

    size_t getObjectCount() const {
        std::lock_guard lock(mutex_);
        return all_objects_.size();
    }

    void clearAllObjects() {
        std::lock_guard lock(mutex_);
        all_objects_.clear();
    }

    // Получение всех объектов для решателя
    std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>> getAllObjects() const {
        std::lock_guard lock(mutex_);
        std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>> objects;
        objects.reserve(all_objects_.size());
        for (const auto& [id, obj] : all_objects_) {
            objects.emplace_back(id, obj);
        }
        return objects;
    }
};
