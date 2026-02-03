#include "include/core/Model.h"
#include "src/DynamicsSystem/FullRocketODE/FullRocketODE.h"
#include "src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"
#include "src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.h"
#include "src/WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.h"
#include "src/WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.h"
#include "src/WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.h"
#include "src/WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.h"
#include "src/WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.h"
#include "PCH.h"
#include "src/PhysicalObjects/Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h"
#include "src/DynamicsSystem/ExtensionModels/Aerodinamics/AeroInput/RocketAeroInput.h"
#include "src/DynamicsSystem/ExtensionModels/Aerodinamics/FullAeroModel/FullAeroModelAdapter.h"
#include "src/SimulationMomento/SimulationMomento.h"
#include "src/utils/SimulationDescriber.h"
#include "src/utils/ContinuationCallback.h"
#include "src/utils/ObjInitParams.h"
#include "src/utils/DataTable/DataTable1D/uploader1D_fromTXT.h"
#include "src/utils/DataTable/DataTable2D/uploader2D_fromTXT.h"
#include "src/utils/IParameterProvider.h"
#include "include/core/GLOBAL_CONFIG.h"
#include "src/PhysicalObjects/SimpleObject/SimpleObject.h"
#include "src/utils/DynamicParametersProviderForFullRocketModel.h"
#include "src/utils/ObjManager/ObjectManager.h"
#include "src/utils/SimulationConfig.h"
#include <cmath>

auto loadDataTable = [](const std::string& filename) {
    try {
        auto loader = uploader1D_fromTXT<GLOBAL_CONFIG::PROJECT_TYPE>(filename);
        return loader.loadFromFile();
    } catch (const std::exception& e) {
        std::cerr << "Error loading data table '" << filename << "': " << e.what() << std::endl;
        throw;
    }
};

int main() {
    try {
        // === ЗАГРУЗКА КОНФИГУРАЦИИ СИМУЛЯЦИИ ИЗ JSON ===
        // Попробуем несколько путей к конфигу
        std::vector<std::string> config_paths = {
            "config/simulation.json",           // Из корня проекта
            "../config/simulation.json",       // Из build директории
            "../../config/simulation_build.json"     // Из Release/Debug поддиректории
        };
        
        SimulationConfig config;
        bool config_loaded = false;
        
        for (const auto& path : config_paths) {
            try {
                config = SimulationConfig::loadFromJsonFile(path);
                config_loaded = true;
                std::cout << "Config loaded from: " << path << std::endl;
                break;
            } catch (const std::exception&) {
                // Попробуем следующий путь
                continue;
            }
        }
        
        if (!config_loaded) {
            throw std::runtime_error("Cannot find simulation config file in any of the expected locations");
        }

        struct ResourceManager {
            std::shared_ptr<SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>> world;
            std::unique_ptr<SimulationDescriber> describer;
            std::unique_ptr<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>> csvDataSaver;
            std::shared_ptr<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> manager;
            std::shared_ptr<MANPAD_V1<GLOBAL_CONFIG::PROJECT_TYPE>> golubka_V1;
            std::shared_ptr<ComponentInterpolationManager<GLOBAL_CONFIG::PROJECT_TYPE>> golubka_V1_interp_mgr;
            ~ResourceManager() {
                golubka_V1.reset();
                manager.reset();
                csvDataSaver.reset();
                world.reset();
            }
        } resource_manager;

        // === ШАГ 1: ИНИЦИАЛИЗАЦИЯ МИРА ===
        resource_manager.world = std::make_shared<SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>>();
        {
            auto wind = std::make_unique<NoWindModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
            auto atmospher = std::make_unique<AtmosphericModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
            auto gravity = std::make_unique<KavendishModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
            auto coriolis = std::make_unique<NoCoriolisForceModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
            auto terrain = std::make_unique<PlaneTerrain<GLOBAL_CONFIG::PROJECT_TYPE>>();
            resource_manager.world->setAtmosphericModel(std::move(atmospher));
            resource_manager.world->setWindModel(std::move(wind));
            resource_manager.world->setGravityModel(std::move(gravity));
            resource_manager.world->setCoriolisEffect(std::move(coriolis));
            resource_manager.world->setTerrain(std::move(terrain));
        }

        // === ШАГ 2: НАСТРОЙКА СОХРАНЕНИЯ ДАННЫХ ===
        resource_manager.describer = std::make_unique<SimulationDescriber>(3);// UTC+3
        resource_manager.describer->setOperatorName(config.operator_name);
        resource_manager.describer->setOdeSolver(config.ode_solver);
        resource_manager.describer->setWorldConfig(config.world_config);
        resource_manager.describer->setDataSaver(config.data_saver);
        resource_manager.describer->setEarthType(config.earth_type);



        resource_manager.csvDataSaver = std::make_unique<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>>();
        resource_manager.csvDataSaver->setStrategy(
            std::make_unique<CsvSaveStrategy<GLOBAL_CONFIG::PROJECT_TYPE>>(config.output_csv));
        resource_manager.csvDataSaver->addTrackedObjs({});

        // === ШАГ 3: ЗАГРУЗКА ДАННЫХ ===
        auto Thrust_x_t_data = loadDataTable(config.thrust_x_path);
        auto Thrust_y_t_data = loadDataTable(config.thrust_y_path);
        auto Thrust_z_t_data = loadDataTable(config.thrust_z_path);
        auto mass_data = loadDataTable(config.mass_path);
        auto Ixx_data = loadDataTable(config.Ixx_path);
        auto Iyy_data = loadDataTable(config.Iyy_path);
        auto Izz_data = loadDataTable(config.Izz_path);

        // === ШАГ 4: КРИТИЧЕСКИ ВАЖНО — СНАЧАЛА СОЗДАТЬ И ИНИЦИАЛИЗИРОВАТЬ МЕНЕДЖЕР ===
        resource_manager.golubka_V1_interp_mgr = std::make_shared<ComponentInterpolationManager<GLOBAL_CONFIG::PROJECT_TYPE>>();
        resource_manager.golubka_V1_interp_mgr->setThrust(
            std::move(Thrust_x_t_data),
            std::move(Thrust_y_t_data),
            std::move(Thrust_z_t_data)
        );

        resource_manager.golubka_V1_interp_mgr->setMass(std::move(mass_data));
        resource_manager.golubka_V1_interp_mgr->setInertia(
            std::move(Ixx_data),
            std::move(Iyy_data),
            std::move(Izz_data)
        );

        // === ШАГ 5: ТОЛЬКО ТЕПЕРЬ МОЖНО СОЗДАВАТЬ ПРОВАЙДЕР (менеджер уже существует!) ===
        auto paramsProvider = std::make_shared<DynamicParametersProviderForFullRocketModel<GLOBAL_CONFIG::PROJECT_TYPE>>(
            resource_manager.golubka_V1_interp_mgr  // ← shared_ptr живёт в resource_manager!
        );

        // === ШАГ 6: ЗАПОЛНЕНИЕ АЭРОДИНАМИЧЕСКИХ ПАРАМЕТРОВ ===
        auto golubka_V1_aero_input = std::make_unique<RocketAeroInput<GLOBAL_CONFIG::PROJECT_TYPE>>();
        golubka_V1_aero_input->rudder_count = 4;
        golubka_V1_aero_input->D_mid = 0.07f;
        golubka_V1_aero_input->L = 0.5f;
        golubka_V1_aero_input->L_har = 0.45f;
        golubka_V1_aero_input->Xdp = 0.4f;
        golubka_V1_aero_input->Xdst = 0.45f;
        golubka_V1_aero_input->L_sum_kr = 0.251f;
        golubka_V1_aero_input->b_0_kr = 0.0355f;
        golubka_V1_aero_input->b_1_kr = 0.0355f;
        golubka_V1_aero_input->hi_st = 0.45f;
        golubka_V1_aero_input->d_kr = 0.01f;
        golubka_V1_aero_input->S_kr = 0.005f;
        golubka_V1_aero_input->l2_kr = 0.251f;
        golubka_V1_aero_input->X_p_kr_st = 0.45f;
        golubka_V1_aero_input->L_sum_p = 0.1f;
        golubka_V1_aero_input->b_0_p = 0.02f;
        golubka_V1_aero_input->b_1_p = 0.02f;
        golubka_V1_aero_input->hi_p = 0.45f;
        golubka_V1_aero_input->d_p = 0.005f;
        golubka_V1_aero_input->S_p = 0.002f;
        golubka_V1_aero_input->l_2_p = 0.4f;
        golubka_V1_aero_input->X_p_kr_p = 0.4f;
        golubka_V1_aero_input->X_ov_p = 0.4f;
        golubka_V1_aero_input->b_sah_p = 0.01f;

        // === ШАГ 7: НАЧАЛЬНЫЕ УСЛОВИЯ (50 м/с под 15°, угловая скорость 5 град/с по тангажу) ===
        auto golubka_V1_init_params = std::make_unique<ObjInitParams<GLOBAL_CONFIG::PROJECT_TYPE>>();
        golubka_V1_init_params->position = config.rocket_init.position.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->velocity = config.rocket_init.velocity.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->eulerAngles = config.rocket_init.euler.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->angularVelocity = config.rocket_init.angular_velocity.cast<GLOBAL_CONFIG::PROJECT_TYPE>();

        // === ШАГ 8: СОЗДАНИЕ СИСТЕМЫ И РАКЕТЫ ===
        auto golubka_V1_system = std::make_unique<FullRocketODE<GLOBAL_CONFIG::PROJECT_TYPE>>(
            paramsProvider,
            resource_manager.world
        );

        // Аэродинамическая модель подключается через интерфейс IAeroModel (DI)
        auto aero_model = std::make_shared<FullAeroModelAdapter<GLOBAL_CONFIG::PROJECT_TYPE>>(
            *golubka_V1_aero_input,
            resource_manager.golubka_V1_interp_mgr
        );

        resource_manager.golubka_V1 = std::make_shared<MANPAD_V1<GLOBAL_CONFIG::PROJECT_TYPE>>(
            std::move(golubka_V1_system),
            std::move(golubka_V1_init_params),
            std::move(golubka_V1_aero_input),
            std::weak_ptr<IParameterProvider<GLOBAL_CONFIG::PROJECT_TYPE>>(resource_manager.golubka_V1_interp_mgr),
            aero_model
        );

        // === ШАГ 9: КРИТИЧЕСКИ ВАЖНО — ПРАВИЛЬНЫЙ ТИП КОЛБЭКА ===
        using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>;
        auto solver = std::make_unique<RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType>>();

        // === ШАГ 10: СОЗДАНИЕ МЕНЕДЖЕРА ОБЪЕКТОВ ===
        resource_manager.manager = std::make_shared<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto golubka_V1_ID = resource_manager.manager->addTrackedObject(resource_manager.golubka_V1);

        // === ШАГ 11: КОЛБЭК С ПРОВЕРКОЙ ВЫСОТЫ ===
        auto continue_callback = [golubka_V1_ID, config](
            const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>& object_manager, const GLOBAL_CONFIG::PROJECT_TYPE current_time
        ) -> bool {
            const auto obj = object_manager->getObjectByID(golubka_V1_ID);
            if (!obj || !obj->isActive()) return false;
            if (current_time > config.max_time) return false;
            const auto& state = obj->getStateSnapshot();
            return state.getPosition().z() >= 0.0f;  // Остановка при ударе о землю
        };



        // === ШАГ 12: ПОЛУЧЕНИЕ ВСЕХ ID ИЗ МЕНЕДЖЕРА ===
        auto all_objects = resource_manager.manager->getAllObjects();
        std::vector<int> ids;
        ids.reserve(all_objects.size());
        for (const auto& [id, obj_weak_ptr] : all_objects) {
            ids.push_back(id);
        }

        resource_manager.describer->setSimulationObjects(ids);

        // === ШАГ 13: ЗАПУСК СИМУЛЯЦИИ ===
        IModel<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType> model(
            std::move(solver),
            resource_manager.world,
            std::move(resource_manager.csvDataSaver),
            std::move(resource_manager.describer),
            resource_manager.manager,
            std::move(continue_callback),
            static_cast<GLOBAL_CONFIG::PROJECT_TYPE>(config.time_step)
        );

        model.run();

        // === ШАГ 14: ЯВНОЕ СОХРАНЕНИЕ ===
        if (resource_manager.csvDataSaver) {
            resource_manager.csvDataSaver->save();
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Main error: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "Unknown error in main" << std::endl;
        return 1;
    }
    return 0;
}