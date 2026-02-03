//
// Created by 4NR_Operator_3 on 12.11.2025.
//

#pragma once
#include "PCH.h"
#include "../utils/ObjManager/IObjectManager.h"
#include "GLOBAL_CONFIG.h"


inline bool isMainRocketActive(const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> &object_manager, const int mainRocID) {
    const auto obj = object_manager->getObjectByID(mainRocID);
    if (!obj || !obj->isActive()) return false;
    const auto& state = obj->getStateSnapshot();
    return state.getPosition().z() >= 0.0f; // Остановка при ударе о землю
}

// можно через фабрику сделать, чтобы условия были кастомными.