//
// Created by 4NR_Operator_3 on 20.11.2025.
//

#pragma once
#include <utility>

#include "PCH.h"
#include "../IDataUploader.h"
#include "../../Interpolation/ILinearInterpolator/LinearInterpolator/LinearInterpolator.h"

/**
 * \brief Класс загрузки из тхт файла 1d таблицы
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \detail Просто считывает данные из файла и все,
 * прикол в конструкторе, из-за особенностей создания пути,он в мейне добавляется,
 * на 1 уровень выше добавляется "../"
 * а также при необходимой интерполяции ,например, F(x), когда нас интересует сила, необходимо в файл txt сначала
 * записывать строку со временными точками, только потом с другой величиной(силой)
 */

template<typename metricType>
class uploader1D_fromTXT final:public IDataUploader<ILinearInterpolator<metricType>> {
protected:
    const std::string filename_;
public:
    explicit uploader1D_fromTXT(std::string filename)
    : filename_(std::move(filename))  {};
    ~uploader1D_fromTXT() override = default;
    std::unique_ptr<ILinearInterpolator<metricType>> loadFromFile() override {
        std::ifstream file(filename_);
        if (!file.is_open()) {throw std::runtime_error("Не могу открыть файл, братик: " + filename_);}

        std::vector<metricType> x_vec;
        std::vector<metricType> y_vec;
        std::string line;

        if (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            std::getline(ss, item, ',');
            while (std::getline(ss, item, ',')) {
                x_vec.push_back(std::stod(item));
            }
        }

        if (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            std::getline(ss, item, ',');
            while (std::getline(ss, item, ',')) {
                y_vec.push_back(std::stod(item));
            }
        }

        if (x_vec.empty() || y_vec.empty()) {
            throw std::runtime_error("Файл пустой, невозможно обработать: " + filename_);
        }

        return std::make_unique<LinearInterpolator<metricType>>(std::move(x_vec), std::move(y_vec));
    }
};



