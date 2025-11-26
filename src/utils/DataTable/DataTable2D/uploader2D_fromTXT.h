//
// Created by 4NR_Operator_3 on 21.11.2025.
//

#pragma once
#include "../IDataUploader.h"
#include "../../Interpolation/IBilinearInterpolator/BilinearInterpolator/BilinearInterpolator.h"

/**
 * \brief Класс загрузки из тхт файла 2d таблицы
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \detail
 * прикол в конструкторе, из-за особенностей создания пути,он в мейне добавляется,
 * на 1 уровень выше добавляется "../"
 */

template<typename metricType>
class uploader2D_fromTXT final:public IDataUploader<metricType,BilinearInterpolator<metricType>> {
protected:
    const std::string filename_;
public:

    explicit uploader2D_fromTXT(const std::string &filename)
    : filename_("../" + filename)  {};
    ~uploader2D_fromTXT() override = default;
 std::unique_ptr<BilinearInterpolator<metricType>> loadFromFile() override {
    std::ifstream file(filename_);
    if (!file.is_open()) {
        throw std::runtime_error("Не могу открыть файл, братик: " + filename_);
    }

    std::vector<metricType> x_vec; // значения m (ось абсцисс)
    std::vector<metricType> y_vec; // значения a (ось ординат)
    std::vector<std::vector<metricType>> z_matrix; // матрица значений

    std::string line;

    // Чтение первой строки с значениями m
    if (std::getline(file, line)) {
        if (size_t pos = line.find('='); pos != std::string::npos) {
            line = line.substr(pos + 1);
        }

        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')) {
            item.erase(0, item.find_first_not_of(" \t"));
            item.erase(item.find_last_not_of(" \t") + 1);
            if (!item.empty()) {
                x_vec.push_back(static_cast<metricType>(std::stod(item)));
            }
        }
    }

    // Пропускаем пустую строку и  строку с названием оси ординат
    std::getline(file, line);
    std::getline(file, line);

    // Чтение остальных строк
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string item;
        std::vector<metricType> row;

        // Первое значение - это a (ось ординат)
        if (std::getline(ss, item, '\t')) {
            item.erase(0, item.find_first_not_of(" \t"));
            item.erase(item.find_last_not_of(" \t") + 1);
            if (!item.empty()) {
                y_vec.push_back(static_cast<metricType>(std::stod(item)));
            }
        }

        // Остальные значения - это данные для матрицы (разделенные запятыми)
        std::string data_part;
        if (std::getline(ss, data_part)) {
            std::stringstream data_ss(data_part);
            while (std::getline(data_ss, item, ',')) {
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);
                if (!item.empty()) {
                    row.push_back(static_cast<metricType>(std::stod(item)));
                }
            }
        }

        if (!row.empty()) {
            z_matrix.push_back(row);
        }
    }

    // Проверки корректности данных
    if (x_vec.empty() || y_vec.empty() || z_matrix.empty()) {
        throw std::runtime_error("Недостаточно данных в файле: " + filename_);
    }

    if (y_vec.size() != z_matrix.size()) {
        throw std::runtime_error("Несоответствие размеров данных в файле: " + filename_);
    }

    for (const auto& row : z_matrix) {
        if (row.size() != x_vec.size()) {
            throw std::runtime_error("Несоответствие количества столбцов в файле: " + filename_);
        }
    }

    return std::make_unique<BilinearInterpolator<metricType>>(
        std::move(x_vec),
        std::move(y_vec),
        std::move(z_matrix)
    );
}
};