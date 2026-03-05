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
#include "src/utils/Interpolation/ComponentInterpolationManager.h"

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
        // === ШАГ 0: ЗАГРУЗКА КОНФИГУРАЦИИ ===
        auto config = SimulationConfig::loadFromJsonFile("../config/simulation.json");

        // Ресурс-менеджер для автоматического освобождения памяти
        struct ResourceManager {
            std::shared_ptr<SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>> world;
            std::unique_ptr<SimulationDescriber> describer;
            std::unique_ptr<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>> csvDataSaver;
            std::shared_ptr<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> manager;
            std::shared_ptr<MANPAD_V1<GLOBAL_CONFIG::PROJECT_TYPE>> golubka_V1;

            // Храним конкретный тип для настройки интерполяторов
            std::shared_ptr<ComponentInterpolationManager<GLOBAL_CONFIG::PROJECT_TYPE>> golubka_V1_interp_mgr_concrete;

            // === УДАЛЕНО: golubka_V1_interp_mgr_base больше не нужен ===
            // shared_ptr<ComponentInterpolationManager> неявно конвертируется в weak_ptr<IParameterProvider>
            // при передаче в конструктор MANPAD_V1 — это безопасно и архитектурно чисто.

            ~ResourceManager() {
                golubka_V1.reset();
                manager.reset();
                csvDataSaver.reset();
                world.reset();
                golubka_V1_interp_mgr_concrete.reset();
                // === УДАЛЕНО: golubka_V1_interp_mgr_base.reset(); ===
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
        resource_manager.describer = std::make_unique<SimulationDescriber>(3); // UTC+3
        resource_manager.describer->setOperatorName(config.operator_name);
        resource_manager.describer->setOdeSolver(config.ode_solver);
        resource_manager.describer->setWorldConfig(config.world_config);
        resource_manager.describer->setDataSaver(config.data_saver);
        resource_manager.describer->setEarthType(config.earth_type);

        resource_manager.csvDataSaver = std::make_unique<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>>();
        resource_manager.csvDataSaver->setStrategy(
            std::make_unique<CsvSaveStrategy<GLOBAL_CONFIG::PROJECT_TYPE>>(config.output_csv));
        resource_manager.csvDataSaver->addTrackedObjs({});

        // === ШАГ 3: ЗАГРУЗКА ДАННЫХ ИЗ ФАЙЛОВ ===
        auto Thrust_x_t_data = loadDataTable(config.thrust_x_path);
        auto Thrust_y_t_data = loadDataTable(config.thrust_y_path);
        auto Thrust_z_t_data = loadDataTable(config.thrust_z_path);
        auto mass_data = loadDataTable(config.mass_path);
        auto Ixx_data = loadDataTable(config.Ixx_path);
        auto Iyy_data = loadDataTable(config.Iyy_path);
        auto Izz_data = loadDataTable(config.Izz_path);
        auto x_com_data = loadDataTable(config.COM_x_path);
        auto y_com_data = loadDataTable(config.COM_y_path);
        auto z_com_data = loadDataTable(config.COM_z_path);

        // === ШАГ 4: СОЗДАНИЕ И НАСТРОЙКА МЕНЕДЖЕРА ИНТЕРПОЛЯЦИИ ===
        // Создаём как конкретный тип для доступа к set-методам
        resource_manager.golubka_V1_interp_mgr_concrete =
            std::make_shared<ComponentInterpolationManager<GLOBAL_CONFIG::PROJECT_TYPE>>();

        resource_manager.golubka_V1_interp_mgr_concrete->setThrust(
            std::move(Thrust_x_t_data),
            std::move(Thrust_y_t_data),
            std::move(Thrust_z_t_data)
        );

        resource_manager.golubka_V1_interp_mgr_concrete->setCOM(
            std::move(x_com_data),
            std::move(y_com_data),
            std::move(z_com_data)
        );

        resource_manager.golubka_V1_interp_mgr_concrete->setMass(std::move(mass_data));

        resource_manager.golubka_V1_interp_mgr_concrete->setInertia(
            std::move(Ixx_data),
            std::move(Iyy_data),
            std::move(Izz_data)
        );

        // === УДАЛЕНО: static_pointer_cast больше не нужен ===
        // golubka_V1_interp_mgr_base удалён — неявная конвертация при передаче в MANPAD_V1

        // === ШАГ 5: СОЗДАНИЕ ПРОВАЙДЕРА ЧЕРЕЗ ФАБРИКУ ===
        auto paramsProvider = std::make_shared<DynamicParametersProvider<GLOBAL_CONFIG::PROJECT_TYPE>>(
            ParameterProviderFactory<GLOBAL_CONFIG::PROJECT_TYPE>::createFromManager(
                resource_manager.golubka_V1_interp_mgr_concrete,  // конкретный тип для factory
                true,   // need_mass
                true,   // need_inertia
                true,   // need_thrust
                true    // need_com
            )
        );

        // === ШАГ 6: НАЧАЛЬНЫЕ УСЛОВИЯ ===
        auto golubka_V1_init_params = std::make_unique<ObjInitParams<GLOBAL_CONFIG::PROJECT_TYPE>>();
        golubka_V1_init_params->position = config.rocket_init.position.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->velocity = config.rocket_init.velocity.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->eulerAngles = config.rocket_init.euler.cast<GLOBAL_CONFIG::PROJECT_TYPE>();
        golubka_V1_init_params->angularVelocity = config.rocket_init.angular_velocity.cast<GLOBAL_CONFIG::PROJECT_TYPE>();

        // === ШАГ 7: ЗАГРУЗКА АЭРОДИНАМИЧЕСКОЙ МОДЕЛИ ===
        auto aero_model = aero::AerodynamicsModel::createFromFile("../config/aerodynamics_rocket_1m.json");

        // === ШАГ 8: СОЗДАНИЕ ODE-СИСТЕМЫ ===
        auto golubka_V1_system = std::make_unique<FullRocketODE<GLOBAL_CONFIG::PROJECT_TYPE>>(
            paramsProvider,
            resource_manager.world,
            aero_model
        );

        // === ШАГ 9: СОЗДАНИЕ ОБЪЕКТА MANPAD_V1 ===
        // Передаём concrete-указатель напрямую — неявная конвертация в weak_ptr<IParameterProvider>
        // Теперь static_pointer_cast работает, т.к. virtual убран из IParameterProvider
        resource_manager.golubka_V1 = std::make_shared<MANPAD_V1<GLOBAL_CONFIG::PROJECT_TYPE>>(
            std::move(golubka_V1_system),
            std::move(golubka_V1_init_params),
            resource_manager.golubka_V1_interp_mgr_concrete  // <-- implicit conversion to weak_ptr<IParameterProvider>
        );

        // === ШАГ 10: НАСТРОЙКА SOLVER ===
        using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>;
        auto solver = std::make_unique<RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType>>();

        // === ШАГ 11: СОЗДАНИЕ МЕНЕДЖЕРА ОБЪЕКТОВ ===
        resource_manager.manager = std::make_shared<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto golubka_V1_ID = resource_manager.manager->addTrackedObject(resource_manager.golubka_V1);

        // === ШАГ 12: КОЛБЭК С ПРОВЕРКОЙ УСЛОВИЙ ОСТАНОВКИ ===
        auto continue_callback = [golubka_V1_ID, config](
            const std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>& object_manager,
            const GLOBAL_CONFIG::PROJECT_TYPE current_time
        ) -> bool {
            const auto obj = object_manager->getObjectByID(golubka_V1_ID);
            if (!obj || !obj->isActive()) return false;
            if (current_time > config.max_time) return false;
            const auto& state = obj->getStateSnapshot();
            const auto height = state.getPosition().z();
            return height >= static_cast<GLOBAL_CONFIG::PROJECT_TYPE>(-0.1);
        };

        // === ШАГ 13: СБОР ID ВСЕХ ОБЪЕКТОВ ДЛЯ ОПИСАНИЯ ===
        auto all_objects = resource_manager.manager->getAllObjects();
        std::vector<int> ids;
        ids.reserve(all_objects.size());
        for (const auto& [id, obj_weak_ptr] : all_objects) {
            ids.push_back(id);
        }
        resource_manager.describer->setSimulationObjects(ids);

        // === ШАГ 14: ЗАПУСК СИМУЛЯЦИИ ===
        IModel<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType> model(
            std::move(solver),
            resource_manager.world,
            std::move(resource_manager.csvDataSaver),
            std::move(resource_manager.describer),
            resource_manager.manager,
            std::move(continue_callback),
            config.time_step
        );

        model.run();

    } catch (const std::exception& e) {
        std::cerr << "Main error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error in main" << std::endl;
        return 1;
    }
    return 0;
}