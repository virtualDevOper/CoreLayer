// //
// // Created by 4NR_Operator_3 on 31.10.2025.


#pragma once
#include "PCH.h"


struct SimulationDescriber {
    int utc_offset_{0};
    std::time_t start_time{0};
    std::string operator_name{"undefined"};
    std::string ode_solver{"undefined"};
    std::string world_config{"undefined"};
    std::string data_saver{"undefined"};
    std::string earth_type{"undefined"};
    std::vector<std::string> simulation_objects{};

    explicit SimulationDescriber(int utc_offset);

    // Методы для установки значений извне
    void setOperatorName(const std::string& name);
    void setOdeSolver(const std::string& solver);
    void setWorldConfig(const std::string& config);
    void setDataSaver(const std::string& saver);
    void setEarthType(const std::string& type);
    void setSimulationObjects(const std::vector<int>& ids);
};