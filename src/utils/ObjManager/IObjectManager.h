//
// Created by 4NR_Operator_3 on 26.01.2026.
//

#pragma once
#include "../../PhysicalObjects/AbstractObject.h"

template<typename metricType>
class IObjectManager {
public:
    virtual ~IObjectManager() = default;
    virtual int addTrackedObject(std::shared_ptr<AbstractObject<metricType>> object)  = 0;
    virtual std::shared_ptr<AbstractObject<metricType>> getObjectByID(int id) const = 0;
    virtual bool removeObjectById(int id)  = 0;
    virtual size_t getObjectCount()  const = 0;
    virtual void clearAllObjects()  = 0;
    virtual std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>> getAllObjects() const = 0;
};