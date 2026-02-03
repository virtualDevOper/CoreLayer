//
// Created by 4NR_Operator_3 on 19.11.2025.
//

#pragma once
#include "PCH.h"
#include "../ILinearInterpolator.h"

template<typename metricType>
class LinearInterpolator final : public ILinearInterpolator<metricType> {
private:
    std::vector<metricType> x_data_;
    std::vector<metricType> y_data_;
public:
    LinearInterpolator(const std::vector<metricType>& x_data,
                      const std::vector<metricType>& y_data)
        : x_data_(x_data), y_data_(y_data) {
        if (x_data_.size() != y_data_.size()) {
            throw std::invalid_argument("таблицы для линейной интерполяции разной длины");
        }
    }

    metricType interpolate(const metricType x) const override {
        if (x_data_.empty() || y_data_.empty()) {
            throw std::runtime_error("Interpolation table is empty");
        }

        auto min_max = std::minmax_element(x_data_.begin(), x_data_.end());
        if (x < *min_max.first) {
            return y_data_.front();  // Экстраполяция вниз
        }
        if (x > *min_max.second) {
            return y_data_.back();   // Экстраполяция вверх
        }

        size_t idx = 0;
        while (idx < x_data_.size() - 1 && x_data_[idx + 1] < x) ++idx;
        if (idx >= x_data_.size() - 1) return y_data_.back();

        const metricType x0 = x_data_[idx];
        const metricType x1 = x_data_[idx + 1];
        const metricType y0 = y_data_[idx];
        const metricType y1 = y_data_[idx + 1];

        return y0 + ((y1 - y0) / (x1 - x0)) * (x - x0);
    }

    const auto& getXData() const { return x_data_; }
    const auto& getYData() const { return y_data_; }
};


