//
// Created by 4NR_Operator_3 on 19.11.2025.
//
#pragma once
#include "../../../../PCH.h"

template<typename metricType>
class IDataTable1D {
public:
    IDataTable1D() = default;
    virtual ~IDataTable1D() = default;
    IDataTable1D(const IDataTable1D&) = delete;
    IDataTable1D& operator=(const IDataTable1D&) = default;
    IDataTable1D(IDataTable1D&&) = default;
    IDataTable1D& operator=(IDataTable1D&&) = default;
    virtual void loadFromFile(const std::string& filename) = 0;
    [[nodiscard]] virtual bool isLoaded() const = 0;
    virtual void loadFromVectors(const std::vector<metricType>& x_axis,
        const std::vector<metricType>& y_data) = 0;
protected:
    bool data_loaded_ = false;
};


