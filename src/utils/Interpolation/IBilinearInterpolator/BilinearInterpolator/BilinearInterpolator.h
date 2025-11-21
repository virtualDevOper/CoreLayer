//
// Created by 4NR_Operator_3 on 19.11.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IBilinearInterpolator.h"

template<typename metricType>
class BilinearInterpolator final : public IBilinearInterpolator<metricType> {
private:
    Eigen::Matrix<metricType, Eigen::Dynamic, 1> x_axis_;
    Eigen::Matrix<metricType, Eigen::Dynamic, 1> y_axis_;
    Eigen::Matrix<metricType, Eigen::Dynamic, Eigen::Dynamic> z_data_;

    static Eigen::Index findIndex(const Eigen::Matrix<metricType, Eigen::Dynamic, 1>& vec, metricType value) {
        for (Eigen::Index i = 0; i < vec.size() - 1; ++i)
            {if (vec(i) <= value && vec(i + 1) >= value) {return i;}}
        return vec.size() - 2;
    }

public:
    BilinearInterpolator(const Eigen::Matrix<metricType, Eigen::Dynamic, 1>& x_axis,
                        const Eigen::Matrix<metricType, Eigen::Dynamic, 1>& y_axis,
                        const Eigen::Matrix<metricType, Eigen::Dynamic, Eigen::Dynamic>& z_data)
        : x_axis_(x_axis), y_axis_(y_axis), z_data_(z_data) {
        if (z_data_.rows() != x_axis_.size() || z_data_.cols() != y_axis_.size()) {
            throw std::invalid_argument("Точка по значениям Z находится вне таблицы");
        }
    }

    metricType interpolate(const metricType x, const metricType y) const override {
        if (x < x_axis_.minCoeff() || x > x_axis_.maxCoeff() ||
            y < y_axis_.minCoeff() || y > y_axis_.maxCoeff()) {
            throw std::out_of_range("Точки интерполяции за пределами таблицы");}

        const Eigen::Index i_low = findIndex(x_axis_, x);
        const Eigen::Index i_high = i_low + 1;
        const Eigen::Index j_low = findIndex(y_axis_, y);
        const Eigen::Index j_high = j_low + 1;

        const metricType x1 = x_axis_(i_low), x2 = x_axis_(i_high);
        const metricType y1 = y_axis_(j_low), y2 = y_axis_(j_high);

        const metricType Q11 = z_data_(i_low, j_low);
        const metricType Q12 = z_data_(i_low, j_high);
        const metricType Q21 = z_data_(i_high, j_low);
        const metricType Q22 = z_data_(i_high, j_high);


        const metricType R1 = ((y2 - y) / (y2 - y1)) * Q11 + ((y - y1) / (y2 - y1)) * Q12;
        const metricType R2 = ((y2 - y) / (y2 - y1)) * Q21 + ((y - y1) / (y2 - y1)) * Q22;

        return ((x2 - x) / (x2 - x1)) * R1 + ((x - x1) / (x2 - x1)) * R2;
    }

    const auto& getXData() const { return x_axis_; }
    const auto& getYData() const { return y_axis_; }
    const auto& getZData() const { return z_data_; }
};


