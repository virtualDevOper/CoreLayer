#include "PCH.h"
#include "include/core/Model.h"
#include "src/DynamicsSystem/FullRocketODE/FullRocketODE.h"
#include "src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"
#include "src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.h"
#include "src/WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.h"
#include "src/WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.h"
#include "src/WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.h"
#include "src/WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.h"
#include "src/WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.h"
#include "src/PhysicalObjects/Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h"
#include "src/SimulationMomento/SimulationMomento.h"
#include "src/utils/SimulationDescriber.h"
#include "src/utils/ContinuationCallback.h"
#include "src/utils/ObjInitParams.h"
#include "src/utils/DataTable/DataTable1D/uploader1D_fromTXT.h"
#include "src/utils/DataTable/DataTable2D/uploader2D_fromTXT.h"
#include "src/utils/ParameterProvider/IParameterProvider.h"
#include "include/core/GLOBAL_CONFIG.h"
#include "src/PhysicalObjects/SimpleObject/SimpleObject.h"
#include "src/utils/ParameterProvider/DynamicParametersProvider.h"
#include "src/utils/ParameterProvider/ParameterProviderFactory.h"
#include "src/utils/ObjManager/ObjectManager.h"
#include "src/utils/SimulationConfig.h"
#include "include/aero_simpi/aerodynamics.h"

#include "include/core/factories/Factories.h"


using json = nlohmann::json;

int main() {
    try {
        // === 1. ЗАГРУЗКА И ВАЛИДАЦИЯ КОНФИГА ===
        nlohmann::json config;
        {
            std::ifstream file("../config/simulation.json");
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open config: ../config/simulation_new.json");
            }
            file >> config;
        }

        Factories::validateConfig(config);

        // === 2. ИНИЦИАЛИЗАЦИЯ ЛОГГЕРА ===
        Factories::loggerSetup(
            config.value("logger_type", "both"),
            config.value("log_dir", "../logs")
        );

        // === 3. СОЗДАНИЕ МИРА ===
        auto world = Factories::createWorld(
            config.at("world_config").get<std::string>()
        );

        // === 4. СОЗДАНИЕ МЕНЕДЖЕРА ОБЪЕКТОВ ===
        auto manager = Factories::createManager(
            "Standard Manager", config
        );

        // === 5. СОЗДАНИЕ ОПИСАТЕЛЯ ===
        auto describer = Factories::createDescriber(
            config.value("describer", "Simple Describer"),
            config, 3
        );

        // === 6. СОЗДАНИЕ СОХРАНЯЛКИ ===
        auto data_saver = Factories::createDataSaver(
            config.at("data_saver").get<std::string>(), config
        );

        // === 7. СОЗДАНИЕ РЕШАТЕЛЯ ===
        using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<
            IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>
        >;
        auto solver = Factories::createSolver<CallbackType>(
            config.at("ode_solver").get<std::string>(), config
        );

        // === 8. СОЗДАНИЕ ОБЪЕКТОВ ИЗ КОНФИГА ===
        for (const auto& device_cfg : config.at("devices")) {
            const int device_id = device_cfg.at("id").get<int>();
            auto device = Factories::createObject(
                device_cfg.at("name").get<std::string>(),
                device_cfg,
                world
            );
            manager->addTrackedObject(std::move(device), device_id);
        }

        // === 9. НАСТРОЙКА ОПИСАТЕЛЯ ===
        {
            const auto all_objects = manager->getAllObjects();
            std::vector<int> ids;
            ids.reserve(all_objects.size());
            for (const auto &id: all_objects | std::views::keys) {
                ids.push_back(id);
            }
            describer->setSimulationObjects(ids);
        }

        // === 10. КОЛБЕК ОСТАНОВКИ ===
        if (!config.contains("stop_conditions")) {
            throw std::runtime_error("Missing 'stop_conditions' in config");
        }

        auto stop_callback = Factories::createStopCallback<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>(
            config["stop_conditions"],
            manager
        );

        // === 11. ЗАПУСК МОДЕЛИ ===
        IModel<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType> model(
            std::move(solver),
            world,
            std::move(data_saver),
            std::move(describer),
            manager,
            std::move(stop_callback),
            config.at("time_step").get<GLOBAL_CONFIG::PROJECT_TYPE>()
        );

        model.run();

        return 0;

    } catch (const std::exception& e) {
        try {
            Logger::getInstance().error("Main error: " + std::string(e.what()));
        } catch (...) {
        }
        std::cerr << "FATAL: " << e.what() << std::endl;
        return 1;
    }
}