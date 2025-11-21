//
// Created by 4NR_Operator_3 on 21.11.2025.
//

#pragma once
#include "../../../PCH.h"
#include "../Interpolation/Interpolator/IInterpolator.h"

template<typename metricType>
class IDataTable {
public:
    IDataTable() = default;
    virtual ~IDataTable() = default;
    IDataTable(const IDataTable&) = delete;
    IDataTable& operator=(const IDataTable&) = default;
    IDataTable(IDataTable&&) = default;
    IDataTable& operator=(IDataTable&&) = default;
    virtual std::unique_ptr<IInterpolator<metricType>> loadFromFile(const std::string& filename) = 0;
};
