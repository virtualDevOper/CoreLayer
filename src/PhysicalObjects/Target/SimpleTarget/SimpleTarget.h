/*
//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../Target.h"
#include "../../../../PCH.h"
#include "../../../DynamicsSystem/IDynamicsSystem.h"

template <typename metricType>
class SimpleTarget: public Target<metricType> {
public:
    explicit SimpleTarget(std::unique_ptr<IDynamicsSystem<metricType>>sys) :
    Target<metricType>(std::move(sys)){};
};
*/



