//
// Created by 4NR_Operator_3 on 19.11.2025.
//
#pragma once
#include "../../../../PCH.h"

template<typename metricType>
class IDataTable2D {
public:
    virtual ~IDataTable2D() = default;

    IDataTable2D(const IDataTable2D& ) = delete;
    IDataTable2D& operator=(const IDataTable2D&) = default;
    IDataTable2D(IDataTable2D&&) = default;
    IDataTable2D& operator=(IDataTable2D&&) = default;
    virtual void loadFromFile(const std::string& filename) = 0;
    [[nodiscard]] virtual bool isLoaded() const = 0;
    virtual void loadFromVectors(const std::vector<metricType>& x_axis,
                                 const std::vector<metricType>& y_axis,
                                 const std::vector<std::vector<metricType>>& z_data) = 0;
protected:
    bool data_loaded_ = false;
};


