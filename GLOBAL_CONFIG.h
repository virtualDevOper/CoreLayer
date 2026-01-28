
#pragma once

namespace GLOBAL_CONFIG {
    using PROJECT_TYPE = float;
    template<typename ObjectTypeManager>
    using STOP_SOLVE_FUNC_TYPE = std::function<bool(const std::shared_ptr<ObjectTypeManager>&)>;
}
