//
// Created by 4NR_Operator_3 on 12.11.2025.
//

#pragma once
#include "../../PCH.h"
#include "../PhysicalObjects/ObjectManager.h"
#include "../../GLOBAL_CONFIG.h"


inline bool isMainRocketActive(const std::shared_ptr<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> &object_manager, const int mainRocID) {
    const auto obj = object_manager->getObjectByID(mainRocID);
    return obj && obj->isActive();
}

// можно через фабрику сделать, чтобы условия были кастомными.