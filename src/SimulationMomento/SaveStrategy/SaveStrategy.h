//
// Created by 4NR_Operator_3 on 06.10.2025.
//
#pragma once
#include <utility>
#include "../StateStorage.h"
#include "../../../PCH.h"
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
        // Открываем файл для записи (перезаписываем)
        std::ofstream file(filename_, std::ios::trunc);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename_);
        }

        try {
            const std::set<std::string> allParamNames = collectAllParamNames(data);
            writeCsvHeader(file, allParamNames);
            writeCsvData(file, data, allParamNames);
            file.flush(); // сразу записываем
        } catch (const std::exception& e) {
            file.close();
            throw std::runtime_error(std::string("CSV write error: ") + e.what());
        }

        file.close();
    }

private:
    std::string filename_;

    std::set<std::string> collectAllParamNames(const std::vector<StateStorage<metricType>>& data) {
        std::set<std::string> paramNames;
        for (const auto& storage : data) {
            for (const auto& snapshot : storage.getStates()) {
                for (const auto& params = snapshot.getParams(); const auto& [paramName, _] : params) {
                    paramNames.insert(paramName);
                }
            }
        }
        return paramNames;
    }

    void writeCsvHeader(std::ofstream& file, const std::set<std::string>& paramNames) {
        file << "object_id,snapshot_index";
        // Добавляем названия параметров в заголовок
        for (const auto& paramName : paramNames) {
            file << "," << escapeCsvField(paramName);
        }
        file << "\n";
    }

    // Записывает данные в CSV формате
    void writeCsvData(std::ofstream& file,
                      const std::vector<StateStorage<metricType>>& data,
                      const std::set<std::string>& paramNames) {
        for (const auto& storage : data) {
            int objectId = storage.getId();
            const auto& states = storage.getStates();
            for (size_t snapshotIndex = 0; snapshotIndex < states.size(); ++snapshotIndex) {
                const auto& snapshot = states[snapshotIndex];
                const auto& params = snapshot.getParams();

                // object_id и snapshot_index
                file << objectId << "," << snapshotIndex;

                // Значения параметров в том же порядке, что и в заголовке
                for (const auto& paramName : paramNames) {
                    file << ",";
                    auto it = params.find(paramName);
                    if (it != params.end()) {
                        metricType value = it->second;
                        // Пропускаем NaN значения (оставляем пустую ячейку)
                        if (!std::isnan(value)) {
                            file << value;
                        }
                    }
                    // Если параметра нет или он имеет NaN - оставляем пустую ячейку
                }
                file << "\n";
            }
        }
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