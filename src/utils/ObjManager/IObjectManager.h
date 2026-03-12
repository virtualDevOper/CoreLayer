//
// Created by 4NR_Operator_3 on 26.01.2026.
//

#pragma once
#include "../../PhysicalObjects/AbstractObject.h"

template<typename metricType>
class IObjectManager {
public:
    virtual ~IObjectManager() = default;

    /**
     * \brief Добавляет объект с явным ID
     * \param object Умный указатель на объект
     * \param id Явный идентификатор объекта
     * \throws std::runtime_error если ID уже занят
     */
    virtual int addTrackedObject(
        std::shared_ptr<AbstractObject<metricType>> object,
        int id
    ) = 0;

    virtual std::shared_ptr<AbstractObject<metricType>> getObjectByID(int id) const = 0;
    virtual std::unordered_map<int, std::weak_ptr<AbstractObject<metricType>>> getAllObjects() const = 0;
    virtual bool removeObject(int id) = 0;
    virtual bool hasObject(int id) const = 0;
    virtual size_t getObjectCount() const = 0;
};