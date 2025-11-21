// //
// // Created by 4NR_Operator_3 on 31.10.2025.


#pragma once
#include "../../PCH.h"


struct SimulationDescriber {
    std::time_t start_time{0};
    std::string operator_name{"undefined"};
    std::string ode_solver{"undefined"};
    std::string world_config{"undefined"};
    std::string data_saver{"undefined"};
    std::vector<std::string> simulation_objects{};
    std::string earth_type{"undefined"};
    double time_step{0.0};
    explicit SimulationDescriber(const int utc_offset) {
        start_time = std::chrono::system_clock::to_time_t(
        std::chrono::system_clock::now() + std::chrono::hours(utc_offset));
     }
};


