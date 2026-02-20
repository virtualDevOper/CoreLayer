#pragma once

/**
 * @file aerodynamics.h
 * @brief Единый заголовочный файл для подключения библиотеки аэродинамики
 * 
 * @example
 * ```cpp
 * #include <aero/aerodynamics.h>
 * 
 * auto model = aero::AerodynamicsModel::create(config);
 * auto output = model->calculate(state);
 * ```
 */

#include "src/types.h"
#include "src/config.h"
#include "src/component.h"
#include "src/parser.h"
#include "src/model.h"

namespace aero {

/**
 * @brief Версия библиотеки
 */
constexpr const char* VERSION = "1.0.0";

/**
 * @brief Проверка совместимости версий
 */
inline bool checkVersion(const char* required) {
    return std::string(VERSION) == std::string(required);
}

} // namespace aero
