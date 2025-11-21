//
// Created by 4NR_Operator_3 on 20.11.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IDataTable1D.h"

template<typename metricType>
class DataTable1D_fromCSV final:public IDataTable1D<metricType> {
public:

    /*
    explicit DataTable1D_fromCSV(const std::vector<metricType>& x,
            const std::vector<metricType>& y) {loadFromVectors(x, y);}

    explicit DataTable1D_fromCSV(const std::string& filename) {loadFromFile(filename);}*/
    ~DataTable1D_fromCSV() override = default;


    /*void loadFromFile(const std::string& filename) override {
        std::ifstream file(filename);
        if (!file.is_open()) {throw std::runtime_error("Не могу открыть файл, братик: " + filename);}

        std::vector<metricType> x_vec;
        std::vector<metricType> y_vec;
        std::string line1;
        std::string line2;

        // Пропускаем заголовок, если есть
        if (std::getline(file, line)) {
            std::stringstream header_ss(line);
            std::string time_header, force_header;
            if (!(std::getline(header_ss, time_header, ',') &&
                  std::getline(header_ss, force_header))) {
                file.seekg(0);
            }
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string time_str, force_str;

            if (std::getline(ss, time_str, ',') && std::getline(ss, force_str)) {
                try {
                    time_vec.push_back(std::stold(time_str));
                    force_vec.push_back(std::stold(force_str));
                } catch (const std::exception& e) {
                    throw std::runtime_error("Error parsing data in file: " + filename);
                }
            }
        }

        if (x_vec.empty() || y_vec.empty()) {
            throw std::runtime_error("No valid data found in file: " + filename);
        }

        loadFromVectors(x_vec, y_vec);
    }





    void loadFromVectors(const std::vector<metricType>& x_axis,
                        const std::vector<metricType>& y_data) override {
        if (x_axis.size() != y_data.size()) {throw std::invalid_argument("2 вектора должны быть одинаковой длины");}
        if (x_axis.size() < 2) {throw std::invalid_argument("в таблице не может быть менее 2 точек");}

        // Проверка монотонности времени
        for (size_t i = 1; i < x_axis.size(); ++i) {
            if (x_axis[i] <= x_axis[i - 1]) {throw std::invalid_argument("Какая-то ошибка в файле времени, оно идет нелинейно");}
        }

        Eigen::Matrix<metricType, Eigen::Dynamic, 1> time_eigen =
            Eigen::Map<const Eigen::Matrix<metricType, Eigen::Dynamic, 1>>(
                x_axis.data(), x_axis.size());
        Eigen::Matrix<metricType, Eigen::Dynamic, 1> force_eigen =
            Eigen::Map<const Eigen::Matrix<metricType, Eigen::Dynamic, 1>>(
                y_data.data(), y_data.size());

        interpolator_ = std::make_unique<LinearInterpolator<metricType>>(time_eigen, force_eigen);
        this->data_loaded_ = true;
    }

    [[nodiscard]] bool isLoaded() const override {
        return this->data_loaded_ && (interpolator_ != nullptr);
    }

    metricType getForce(metricType time) const {
        if (!isLoaded()) {
            throw std::runtime_error("ForceTimeTable data not loaded");
        }

        if (time < getTimeMin() || time > getTimeMax()) {
            return static_cast<metricType>(0);
        }

        return interpolator_->interpolate(time) * G;
    }

    metricType getTimeMin() const {
        if (!isLoaded()) throw std::runtime_error("ForceTimeTable data not loaded");
        return interpolator_->getXData().minCoeff();
    }

    metricType getTimeMax() const {
        if (!isLoaded()) throw std::runtime_error("ForceTimeTable data not loaded");
        return interpolator_->getXData().maxCoeff();
    }

    size_t getDataSize() const {
        if (!isLoaded()) throw std::runtime_error("ForceTimeTable data not loaded");
        return interpolator_->getXData().size();
    }

    std::pair<std::vector<metricType>, std::vector<metricType>> getRawData() const {
        if (!isLoaded()) throw std::runtime_error("ForceTimeTable data not loaded");

        const auto& time_data = interpolator_->getXData();
        const auto& force_data = interpolator_->getYData();

        std::vector<metricType> time_vec(time_data.data(),
                                       time_data.data() + time_data.size());
        std::vector<metricType> force_vec(force_data.data(),
                                        force_data.data() + force_data.size());

        return {std::move(time_vec), std::move(force_vec)};
    }*/

    /*
    bool isTimeInRange(metricType time) const {
        if (!isLoaded()) return false;
        return (time >= getTimeMin() && time <= getTimeMax());
    }
*/
};



