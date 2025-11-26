//
// Created by 4NR_Operator_3 on 19.11.2025.
//

#pragma once
#include "../../../../../PCH.h"
#include "../IBilinearInterpolator.h"


/**
 * \brief Класс интерполяции 2Д таблицы
 *
 * \tparam metricType Тип данных для метрических величин
 *
 * \detail Вот тут диллема сложноватая,интерполирование работает, когда
 * вся таблица равномерно распределена(существует какая- никакая линейность).
 * Однако, в АД с этим намного сложнее, поэтому интерполяция может быть не такой точной,
 * как хотелось бы.(Пример следующий: чекни файл Cx_a_m.txt, там при изменении угла отклонения
 * значение сопротивления может вообще не меняться, а может очень сильно меняться)
 */

template<typename metricType>
class BilinearInterpolator final : public IBilinearInterpolator<metricType> {
private:
    std::vector<metricType> x_axis_;
    std::vector<metricType> y_axis_;
    std::vector<std::vector<metricType>> z_data_;

    static size_t findIndex(const std::vector<metricType>& vec, metricType value) {
        for (size_t i = 0; i < vec.size() - 1; ++i) {
            if (vec[i] <= value && vec[i + 1] >= value) {
                return i;
            }
        }
        return vec.size() - 2;
    }

public:
    BilinearInterpolator(const std::vector<metricType>& x_axis,
                        const std::vector<metricType>& y_axis,
                        const std::vector<std::vector<metricType>>& z_data)
        : x_axis_(x_axis), y_axis_(y_axis), z_data_(z_data) {
        if (z_data_.size() != y_axis_.size() ||
            std::any_of(z_data_.begin(), z_data_.end(),
                [&](const auto& row) { return row.size() != x_axis_.size(); })) {
            throw std::invalid_argument("Размеры данных не согласованы");
        }
    }

    metricType interpolate(const metricType x, const metricType y) const override {
        const auto x_min_max = std::minmax_element(x_axis_.begin(), x_axis_.end());
        const auto y_min_max = std::minmax_element(y_axis_.begin(), y_axis_.end());

        if (x < *x_min_max.first || x > *x_min_max.second ||
            y < *y_min_max.first || y > *y_min_max.second) {
            throw std::out_of_range("Точки интерполяции за пределами таблицы");
        }

        const size_t i_low = findIndex(x_axis_, x);
        const size_t i_high = i_low + 1;
        const size_t j_low = findIndex(y_axis_, y);
        const size_t j_high = j_low + 1;

        const metricType x1 = x_axis_[i_low], x2 = x_axis_[i_high];
        const metricType y1 = y_axis_[j_low], y2 = y_axis_[j_high];

        const metricType Q11 = z_data_[j_low][i_low];
        const metricType Q12 = z_data_[j_low][i_high];
        const metricType Q21 = z_data_[j_high][i_low];
        const metricType Q22 = z_data_[j_high][i_high];

        const metricType R1 = ((y2 - y) / (y2 - y1)) * Q11 + ((y - y1) / (y2 - y1)) * Q12;
        const metricType R2 = ((y2 - y) / (y2 - y1)) * Q21 + ((y - y1) / (y2 - y1)) * Q22;

        return ((x2 - x) / (x2 - x1)) * R1 + ((x - x1) / (x2 - x1)) * R2;
    }

    const auto& getXData() const { return x_axis_; }
    const auto& getYData() const { return y_axis_; }
    const auto& getZData() const { return z_data_; }
};


