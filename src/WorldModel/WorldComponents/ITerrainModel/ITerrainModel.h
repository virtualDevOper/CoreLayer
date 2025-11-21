//
// Created by 4NR_Operator_3 on 17.09.2025.
//

#pragma once

template <typename metricType>
class ITerrainModel {
public:
    virtual ~ITerrainModel() = default;
    virtual metricType getHeight(metricType x, metricType y) const = 0;
    [[nodiscard]] virtual bool hasHeightMap() const = 0;
    //в дальнейшем расширяется так, что можно загрузить карту высот определенного формата
};




