//
// Created by 4NR_Operator_3 on 06.10.2025.
//
#pragma once
#include <utility>
#include "../StateStorage.h"
#include "PCH.h"
template <typename metricType>
class SaveStrategy {
public:
    virtual ~SaveStrategy() = default;
    virtual void save(const std::vector<StateStorage<metricType>>& data) = 0;
};
template <typename metricType>
class CsvSaveStrategy final : public SaveStrategy<metricType> {
public:
    explicit CsvSaveStrategy(std::string  filename): filename_(std::move(filename)) {}

    void save(const std::vector<StateStorage<metricType>>& data) override {
        // Для каждого объекта создаём отдельный файл results_data/simulation_data_<id>.txt
        try {
            for (const auto& storage : data) {
                saveSingleStorage(storage);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("CSV write error: ") + e.what());
        }
    }

private:
    std::string filename_;

    // Строим путь вида <dir>/simulation_data_<id>.txt
    std::string buildFilenameForId(int objectId) const {
        std::string base = filename_;
        // Отделяем директорию
        std::string dir;
        auto pos = base.find_last_of("/\\");
        if (pos != std::string::npos) {
            dir = base.substr(0, pos);
        } else {
            dir = ".";
        }
        return dir + "/simulation_data_" + std::to_string(objectId) + ".txt";
    }

    void saveSingleStorage(const StateStorage<metricType>& storage) {
        int objectId = storage.getId();
        const auto& states = storage.getStates();
        if (states.empty()) {
            return;
        }

        // Собираем множество имён параметров для данного объекта
        std::set<std::string> paramNames;
        for (const auto& snapshot : states) {
            for (const auto& params = snapshot.getParams(); const auto& [paramName, _] : params) {
                paramNames.insert(paramName);
            }
        }

        // Отдельно выделяем time, чтобы поставить его первым
        bool hasTime = paramNames.erase("time") > 0;

        const std::string outName = buildFilenameForId(objectId);
        std::ofstream file(outName, std::ios::trunc);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + outName);
        }

        // Заголовок: time, затем остальные параметры в алфавитном порядке
        bool first = true;
        if (hasTime) {
            file << "time";
            first = false;
        }
        for (const auto& paramName : paramNames) {
            if (first) {
                file << escapeCsvField(paramName);
                first = false;
            } else {
                file << "," << escapeCsvField(paramName);
            }
        }
        file << "\n";

        // Данные
        for (const auto& snapshot : states) {
            const auto& params = snapshot.getParams();

            // time
            bool firstField = true;
            if (hasTime) {
                auto it = params.find("time");
                metricType value = (it != params.end()) ? it->second : std::numeric_limits<metricType>::quiet_NaN();
                if (!std::isnan(value)) {
                    file << value;
                }
                firstField = false;
            }

            // остальные параметры
            for (const auto& paramName : paramNames) {
                if (firstField) {
                    // первый параметр в строке (если нет времени)
                    firstField = false;
                } else {
                    file << ",";
                }
                auto it = params.find(paramName);
                if (it != params.end()) {
                    metricType value = it->second;
                    if (!std::isnan(value)) {
                        file << value;
                    }
                }
            }
            file << "\n";
        }

        file.close();
    }

    // Экранирует поле CSV если нужно
    std::string escapeCsvField(const std::string& field) {
        // Если поле содержит запятые, кавычки или переносы строк - экранируем
        if (field.find(',') != std::string::npos ||
            field.find('"') != std::string::npos ||
            field.find('\n') != std::string::npos) {
            std::string escaped = "\"";
            for (char c : field) {
                if (c == '"') {
                    escaped += "\"\"";
                } else {
                    escaped += c;
                }
            }
            escaped += "\"";
            return escaped;
        }
        return field;
    }
};
/*class DatabaseSaveStrategy : public SaveStrategy<metricType> { ... };
class MemorySaveStrategy : public SaveStrategy<metricType> { ... };*/