//
// Created by 4NR_Operator_3 on 28.10.2025.
//

#pragma once
#include "IObjectManager.h"

/**
 * \brief Класс для отслеживания динамических объектов
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \details Этот класс не хранит параметры динамических объектов, такие как координаты
 * этот класс отслеживает жизненный цикл объектов и имеет ссылки на них(а вот то, что внутри абстрактного ЛА - уже в ЛА смотри).
 * Скорее всего в ЛА будет решаться задача динамики полета с датчиков, так что смотри по ситуации.
 */

template<typename metricType>
class ObjectManager final : public IObjectManager<metricType> {
private:
    std::unordered_map<int, std::shared_ptr<AbstractObject<metricType>>> all_objects_;
    mutable std::mutex mutex_;

public:
    ObjectManager() = default;
    ~ObjectManager() override = default;

    int addTrackedObject(
        std::shared_ptr<AbstractObject<metricType>> object,
        int id
    ) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Проверяем, не занят ли уже этот ID
        if (all_objects_.contains(id)) {
            throw std::runtime_error(
                "ObjectManager: cannot add object with ID " + std::to_string(id) +
                " — ID already occupied"
            );
        }

        all_objects_.emplace(id, std::move(object));
        return id;
    }

    std::shared_ptr<AbstractObject<metricType>> getObjectByID(int id) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = all_objects_.find(id);
        if (it != all_objects_.end()) {
            return it->second;
        }
        return nullptr;
    }

    std::unordered_map<int, std::weak_ptr<AbstractObject<metricType>>> getAllObjects() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        std::unordered_map<int, std::weak_ptr<AbstractObject<metricType>>> result;
        result.reserve(all_objects_.size());
        for (const auto& [id, obj] : all_objects_) {
            result.emplace(id, obj);
        }
        return result;
    }

    bool removeObject(int id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        return all_objects_.erase(id) > 0;
    }

    bool hasObject(int id) const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return all_objects_.count(id) > 0;
    }

    size_t getObjectCount() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return all_objects_.size();
    }
};