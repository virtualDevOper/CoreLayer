#include "PCH.h"
#include "RungeKutta4Solver.h"

// Explicit template instantiation
template class RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>>;