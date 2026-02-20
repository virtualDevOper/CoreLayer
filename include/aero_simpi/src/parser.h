#pragma once

#include "config.h"
#include <string>
#include <memory>

namespace aero {

/**
 * @brief Интерфейс загрузчика конфигурации (SOLID: Dependency Inversion)
 */
class IConfigLoader {
public:
    virtual ~IConfigLoader() = default;
    
    /**
     * @brief Загрузка конфигурации из источника
     * @return Загруженная конфигурация
     * @throws ConfigError если конфигурация невалидна
     */
    virtual AeroConfig load() const = 0;
};

/**
 * @brief Загрузчик конфигурации из JSON файла
 */
class JsonConfigLoader : public IConfigLoader {
private:
    std::string filepath_;
    
public:
    explicit JsonConfigLoader(const std::string& filepath);
    AeroConfig load() const override;
};

/**
 * @brief Парсер JSON (реализация через nlohmann/json)
 */
class JsonParser {
public:
    static AeroConfig parse(const std::string& json_content);
    static AeroConfig parseFile(const std::string& filepath);
};

} // namespace aero
