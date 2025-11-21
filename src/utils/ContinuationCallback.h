/*//
// Created by 4NR_Operator_3 on 12.11.2025.
//

#pragma once
#include "../../PCH.h"
#include "../PhysicalObjects/ObjectManager.h"

inline bool isMainRocKetActive(const std::shared_ptr<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> &object_manager, const int mainRocID) {
    return object_manager->getObjectByID(mainRocID)->isActive();
}*/