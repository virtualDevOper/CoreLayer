
#pragma once

#include "PCH.h"

namespace GLOBAL_CONFIG {
    using PROJECT_TYPE = double;
    template<typename ObjectTypeManager>
    using STOP_SOLVE_FUNC_TYPE = std::function<bool(const std::shared_ptr<ObjectTypeManager>&,const GLOBAL_CONFIG::PROJECT_TYPE)>;
}
