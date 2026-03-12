#pragma once
#include "PCH.h"
#include "../utils/ObjManager/IObjectManager.h"
#include "GLOBAL_CONFIG.h"

/**
 * @brief Проверка: время ещё не вышло
 */
inline bool isTimeOk(GLOBAL_CONFIG::PROJECT_TYPE current_time, 
                     GLOBAL_CONFIG::PROJECT_TYPE max_time) {
    return current_time < max_time;
}

/**
 * @brief Проверка: объект активен
 */
inline bool isObjectActive(
    const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>& object_manager,
    int object_id) 
{
    auto obj = object_manager->getObjectByID(object_id);
    return obj && obj->isActive();
}

/**
 * @brief Проверка: высота в норме (не упал)
 */
inline bool isHeightOk(
    const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>& object_manager,
    int object_id,
    GLOBAL_CONFIG::PROJECT_TYPE min_height_value) 
{
    auto obj = object_manager->getObjectByID(object_id);
    if (!obj) return false;
    
    const auto& state = obj->getStateSnapshot();
    return state.getPosition().z() > min_height_value;
}

/**
 * @brief Колбэк продолжения симуляции
 * 
 * ЛОГИКА (как вы хотели):
 * - "OR"  : остановить, если ХОТЯ БЫ ОДНО условие нарушено
 *           (продолжать, только если ВСЕ условия выполнены)
 * - "AND" : остановить, только если ВСЕ условия нарушены  
 *           (продолжать, если ХОТЯ БЫ ОДНО условие выполнено)
 * 
 * @return true = продолжать, false = остановить
 */
inline bool simulationContinueCallback(
    const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>& object_manager,
    GLOBAL_CONFIG::PROJECT_TYPE current_time,
    GLOBAL_CONFIG::PROJECT_TYPE max_time,
    int main_object_id,
    GLOBAL_CONFIG::PROJECT_TYPE min_height_value,
    const std::string& logic = "OR")  // "OR" или "AND"
{
    // Проверяем каждое условие
    bool time_ok = isTimeOk(current_time, max_time);
    bool object_active = isObjectActive(object_manager, main_object_id);
    bool height_ok = isHeightOk(object_manager, main_object_id, min_height_value);
    
    if (logic == "OR") {
        // OR: продолжать ТОЛЬКО если ВСЕ условия выполнены
        // (остановить, если ХОТЯ БЫ ОДНО нарушено)
        return time_ok && object_active && height_ok;
        
    } else { // AND
        // AND: продолжать если ХОТЯ БЫ ОДНО условие выполнено
        // (остановить, только если ВСЕ нарушены)
        return time_ok || object_active || height_ok;
    }
}