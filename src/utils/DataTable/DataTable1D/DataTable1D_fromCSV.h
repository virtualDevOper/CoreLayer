//
// Created by 4NR_Operator_3 on 20.11.2025.
//

#pragma once
#include "../../../../PCH.h"
#include "../IDataTable.h"

template<typename metricType>
class DataTable1D_fromCSV final:public IDataTable<metricType> {
public:

    explicit DataTable1D_fromCSV(const std::string& filename) {loadFromFile(filename);}
    ~DataTable1D_fromCSV() override = default;

    std::unique_ptr<IInterpolator<metricType>> loadFromFile(const std::string& filename) override {
        std::ifstream file(filename);
        if (!file.is_open()) {throw std::runtime_error("Не могу открыть файл, братик: " + filename);}

        std::vector<metricType> x_vec = {0};
        std::vector<metricType> y_vec = {0};
        std::string line1;
        std::string line2;
        //считываем первую строку и вторую, разделяя их по запятой, имя в начале игнорируем по идее а пото просто записываем в 2 вектора



        if (x_vec.empty() || y_vec.empty()) {
            throw std::runtime_error("Файл пустой, невозможно обработать: " + filename);
        }

        return std::make_unique<IInterpolator<metricType>>(std::move(x_vec), std::move(y_vec));
    }

};



