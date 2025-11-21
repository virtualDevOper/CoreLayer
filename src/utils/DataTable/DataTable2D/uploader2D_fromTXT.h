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
class uploader2D_fromTXT final:public IDataUploader<metricType> {
protected:
    const std::string filename_;
public:
    explicit uploader2D_fromTXT(const std::string &filename)
    : filename_("../" + filename)  {};
    ~uploader2D_fromTXT() override = default;
  /*
  std::unique_ptr<BilinearInterpolator<metricType>> loadFromFile() override {
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

        return std::make_unique<BilinearInterpolator<metricType>>(std::move(x_vec), std::move(y_vec));
    }
*/
};