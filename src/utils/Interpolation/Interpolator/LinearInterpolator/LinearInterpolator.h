//
// Created by 4NR_Operator_3 on 19.11.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IInterpolator.h"

template<typename metricType>
class LinearInterpolator final : public IInterpolator<metricType> {
private:
    Eigen::Matrix<metricType, Eigen::Dynamic, 1> x_data_;
    Eigen::Matrix<metricType, Eigen::Dynamic, 1> y_data_;

public:
    LinearInterpolator(const Eigen::Matrix<metricType, Eigen::Dynamic, 1>& x_data,
                      const Eigen::Matrix<metricType, Eigen::Dynamic, 1>& y_data)
        : x_data_(x_data), y_data_(y_data) {
        if (x_data_.size() != y_data_.size()) {
            throw std::invalid_argument("таблицы для линейной интерполяции разной длины");
        }
    }

    metricType interpolate(const metricType x) const override {
        if (x < x_data_.minCoeff() || x > x_data_.maxCoeff()) {throw std::invalid_argument("значение интерполируемой переменной вне таблицы");}
        Eigen::Index idx = 0;
        while (idx < x_data_.size() - 1 && x_data_(idx + 1) < x) ++idx;
        if (idx >= x_data_.size() - 1) return y_data_.tail(1)(0);
        const metricType x0 = x_data_(idx);
        const metricType x1 = x_data_(idx + 1);
        const metricType y0 = y_data_(idx);
        const metricType y1 = y_data_(idx + 1);
        return y0 + ((y1 - y0) / (x1 - x0)) * (x - x0);
    }

    const auto& getXData() const { return x_data_; }
    const auto& getYData() const { return y_data_; }
};


