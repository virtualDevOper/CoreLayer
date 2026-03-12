#include "PCH.h"
#include "RungeKutta4Solver.h"


using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<
    IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>
>;
template class RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType>;

template class RungeKutta4Solver<double, std::function<bool(std::shared_ptr<IObjectManager<double>>, double)>>;