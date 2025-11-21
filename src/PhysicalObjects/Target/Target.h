/*
//
// Created by 4NR_Operator_3 on 29.09.2025.
//

#pragma once
#include "../../../PCH.h"
#include "../AbstractObject.h"


template <typename metricType>
class Target : public AbstractObject<metricType>{
public:
    explicit Target(std::unique_ptr<IDynamicsSystem<metricType>>sys) :
    AbstractObject<metricType>(std::move(sys)){};
// рассматривается чисто в земной системе координат
// вот тут пока что вообще не знаю, по идее нужно сделать такую же систему ОДУ, как и в DynamicSystem, решая систему ОДУ для каждого из микрочелика,
//     !!!!! можно просто унаследоваться от DynamicSystem и хранить также, как и в ракете, тут просто параметры будут по типу конструкции и тд.
};
*/



