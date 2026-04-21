#pragma once

#include "../PCH.h"

#include "../../../src/WorldModel/AbstractWorldModel.h"
#include "../../../src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.h"
#include "logging/Logger.h"
#include "OdeSolver/ODESolver.h"
#include "SimulationMomento/SimulationMomento.h"
#include "utils/SimulationDescriber.h"
#include "utils/ObjManager/ObjectManager.h"
#include "utils/Interpolation/ComponentInterpolationManager.h"
#include "utils/ParameterProvider/DynamicParametersProvider.h"
#include "utils/ParameterProvider/ParameterProviderFactory.h"
#include "utils/DataTable/DataTable1D/uploader1D_fromTXT.h"
#include "utils/DataTable/DataTable2D/uploader2D_fromTXT.h"
#include "utils/ParameterProvider/IParameterProvider.h"
#include "DynamicsSystem/IDynamicsSystem.h"
#include "DynamicsSystem/Aerodynamics/NetAeroModel.h"
#include "DynamicsSystem/FullRocketODE/FullRocketODE.h"
#include "DynamicsSystem/NetProjectile3DOF/NetProjectile3DOF.h"
#include "WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.h"
#include "WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.h"
#include "WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.h"
#include "WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.h"
#include "WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.h"




#include "OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"
#include "PhysicalObjects/Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h"
#include "PhysicalObjects/Aircraft/Missle/UnGuidedMissle/NetProjectile.h"
#include "PhysicalObjects/SimpleObject/SimpleObject.h"
#include "utils/DataTable/DataTable1D/uploader1D_fromTXT.h"
#include "utils/DataTable/DataTable2D/uploader2D_fromTXT.h"

// ============================================================================
// 1. ТИПЫ И ENUMS(Они ни на что не влияют, просто для понимания, что уже реализовано)
// ============================================================================
enum class LoggerType { File, Console, Both };
enum class WorldType { SimpleFlatEarth };
enum class DescriberType { Simple, Detailed };
enum class SaverType { CSV };
enum class DeviceType { Manpad, SimpleTarget, NetProjectile };
enum class SolverType { RungeKutta4 };

// Тип колбека остановки
template<typename ManagerType>
using StopCallbackType = std::function<bool(
    const std::shared_ptr<ManagerType>&,
    GLOBAL_CONFIG::PROJECT_TYPE
)>;

// ============================================================================
// 2. КЛАСС ФАБРИК
// ============================================================================
class Factories {
public:

    static void validateConfig(const nlohmann::json& config) {
        if (!config.contains("operator_name")) {
            throw std::runtime_error("Config validation: missing 'operator_name'");
        }

        if (!config.contains("time_step")) {
            throw std::runtime_error("Config validation: missing 'time_step'");
        }

        if (!config.contains("devices") || !config["devices"].is_array()) {
            throw std::runtime_error("Config validation: 'devices' must be a non-empty array");
        }

        if (config.contains("stop_conditions")) {
            validateStopConditions(config["stop_conditions"]);
        }
    }

    static void validateStopConditions(const nlohmann::json& stop_config) {
        if (!stop_config.contains("main_object_id")) {
            throw std::runtime_error(
                "StopConditions validation: missing required field 'main_object_id'"
            );
        }

        if (!stop_config["main_object_id"].is_number_integer()) {
            throw std::runtime_error(
                "StopConditions validation: 'main_object_id' must be an integer"
            );
        }

        bool has_max_time = stop_config.contains("max_time");
        bool has_min_height = stop_config.contains("min_height");

        if (!has_max_time && !has_min_height) {
            throw std::runtime_error(
                "StopConditions validation: at least one condition must be specified "
                "('max_time' or 'min_height')"
            );
        }

        if (has_max_time) {
            if (!stop_config["max_time"].is_number()) {
                throw std::runtime_error(
                    "StopConditions validation: 'max_time' must be a number"
                );
            }
            if (stop_config["max_time"].get<GLOBAL_CONFIG::PROJECT_TYPE>() <= 0) {
                throw std::runtime_error(
                    "StopConditions validation: 'max_time' must be positive"
                );
            }
        }

        if (has_min_height) {
            const auto& height_config = stop_config["min_height"];
            if (height_config.is_number()) {
                if (height_config.get<GLOBAL_CONFIG::PROJECT_TYPE>() > 1000 ||
                    height_config.get<GLOBAL_CONFIG::PROJECT_TYPE>() < -1000) {
                    Logger::getInstance().warning(
                        "StopConditions: min_height value seems unusual: " +
                        std::to_string(height_config.get<GLOBAL_CONFIG::PROJECT_TYPE>())
                    );
                }
            }
            else if (height_config.is_object()) {
                if (height_config.contains("object_id") &&
                    !height_config["object_id"].is_number_integer()) {
                    throw std::runtime_error(
                        "StopConditions validation: 'min_height.object_id' must be an integer"
                    );
                }
                if (!height_config.contains("value") || !height_config["value"].is_number()) {
                    throw std::runtime_error(
                        "StopConditions validation: 'min_height.value' must be a number"
                    );
                }
            }
            else {
                throw std::runtime_error(
                    "StopConditions validation: 'min_height' must be a number or an object"
                );
            }
        }

        if (stop_config.contains("logic")) {
            const auto logic = stop_config["logic"].get<std::string>();
            if (logic != "AND" && logic != "OR") {
                throw std::runtime_error(
                    "StopConditions validation: 'logic' must be 'AND' or 'OR', got: " + logic
                );
            }
        }
    }

    static void loggerSetup(const std::string& name, const std::string& log_dir) {
        if (name == "FILE" || name == "file") {
            Logger::getInstance().configure(true, false, log_dir);
        } else if (name == "CONSOLE" || name == "console") {
            Logger::getInstance().configure(false, true, log_dir);
        } else if (name == "BOTH" || name == "both") {
            Logger::getInstance().configure(true, true, log_dir);
        } else {
            throw std::runtime_error("Unknown logger type: " + name);
        }
    }

    static std::shared_ptr<AbstractWorldModel<GLOBAL_CONFIG::PROJECT_TYPE>>
    createWorld(const std::string& name) {
        if (name == "SimpleFlatEarth") {
            auto world = std::make_shared<SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>>();
            world->setAtmosphericModel(
                std::make_unique<AtmosphericModel<GLOBAL_CONFIG::PROJECT_TYPE>>()
            );
            world->setWindModel(
                std::make_unique<NoWindModel<GLOBAL_CONFIG::PROJECT_TYPE>>()
            );
            world->setGravityModel(
                std::make_unique<KavendishModel<GLOBAL_CONFIG::PROJECT_TYPE>>()
            );
            world->setCoriolisEffect(
                std::make_unique<NoCoriolisForceModel<GLOBAL_CONFIG::PROJECT_TYPE>>()
            );
            world->setTerrain(
                std::make_unique<PlaneTerrain<GLOBAL_CONFIG::PROJECT_TYPE>>()
            );
            return world;
        }
        throw std::runtime_error("Unknown World type: " + name);
    }

    static std::unique_ptr<SimulationDescriber>
    createDescriber(const std::string& name,
                    const nlohmann::json& config,
                    int utc_offset) {
        if (name == "Simple Describer") {
            auto describer = std::make_unique<SimulationDescriber>();
            describer->set_offcet(utc_offset);

            if (config.contains("operator_name")) {
                describer->setOperatorName(config.at("operator_name").get<std::string>());
            }
            if (config.contains("ode_solver")) {
                describer->setOdeSolver(config.at("ode_solver").get<std::string>());
            }
            if (config.contains("world_config")) {
                describer->setWorldConfig(config.at("world_config").get<std::string>());
            }
            if (config.contains("data_saver")) {
                describer->setDataSaver(config.at("data_saver").get<std::string>());
            }

            return describer;
        }
        throw std::runtime_error("Unknown Describer type: " + name);
    }


    static std::unique_ptr<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>>
    createDataSaver(const std::string& name, const nlohmann::json& config) {
        if (name == "CSV DataSaver") {
            auto saver = std::make_unique<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>>();
            if (config.contains("output_csv")) {
                saver->setStrategy(
                    std::make_unique<CsvSaveStrategy<GLOBAL_CONFIG::PROJECT_TYPE>>(
                        config.at("output_csv").get<std::string>()
                    )
                );
            } else {
                throw std::runtime_error("CSV DataSaver: missing 'output_csv' path");
            }
            return saver;
        }
        throw std::runtime_error("Unknown DataSaver type: " + name);
    }


    static std::shared_ptr<IObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>
    createManager(const std::string& name, const nlohmann::json& /*config*/) {
        if (name == "Standard Manager") {
            return std::make_shared<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>();
        }
        throw std::runtime_error("Unknown Manager type: " + name);
    }


    template<typename CallbackType>
    static std::unique_ptr<ODESolver<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType>>
    createSolver(const std::string& name, const nlohmann::json& /*config*/) {
        if (name == "RungeKutta4") {
            return std::make_unique<RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, CallbackType>>();
        }
        throw std::runtime_error("Unknown Solver type: " + name);
    }


    template<typename ObjectTypeManager>
    static auto createStopCallback(
        const nlohmann::json& stop_conditions,
        std::shared_ptr<ObjectTypeManager> manager)
    {
        GLOBAL_CONFIG::PROJECT_TYPE max_time = stop_conditions.value("max_time",
            GLOBAL_CONFIG::PROJECT_TYPE(100.0));
        int main_object_id = stop_conditions.value("main_object_id", 0);

        GLOBAL_CONFIG::PROJECT_TYPE min_height_value = 0.0;
        if (stop_conditions.contains("min_height")) {
            const auto& mh = stop_conditions["min_height"];
            if (mh.is_number()) {
                min_height_value = mh.get<GLOBAL_CONFIG::PROJECT_TYPE>();
            } else if (mh.is_object()) {
                min_height_value = mh.value("value",
                    GLOBAL_CONFIG::PROJECT_TYPE(-0.1));
            }
        }

        std::string logic = stop_conditions.value("logic", std::string("OR"));

        // Возвращаем лямбду с захваченными параметрами
        return [manager, max_time, main_object_id, min_height_value, logic](
            const std::shared_ptr<ObjectTypeManager>&,
            GLOBAL_CONFIG::PROJECT_TYPE current_time)
        {
            return simulationContinueCallback(
                manager,
                current_time,
                max_time,
                main_object_id,
                min_height_value,
                logic
            );
        };
    }


    static std::shared_ptr<AbstractObject<GLOBAL_CONFIG::PROJECT_TYPE>>
    createObject(const std::string& name,
                 const nlohmann::json& device_config,
                 const std::shared_ptr<AbstractWorldModel<GLOBAL_CONFIG::PROJECT_TYPE>>& world) {

        if (name == "manpad") return createManpad(device_config, world);
        if (name == "simple_object") return createSimpleObject(device_config, world);
        if (name == "net_projectile") return createNetProjectile(device_config, world);
        throw std::runtime_error("Unknown device type: " + name);
    }

private:

    static Eigen::Vector3<GLOBAL_CONFIG::PROJECT_TYPE>
    parseVector3(const nlohmann::json& arr) {
        using T = GLOBAL_CONFIG::PROJECT_TYPE;
        if (!arr.is_array() || arr.size() != 3) {
            throw std::runtime_error("Vector3 JSON must be array of 3 numbers");
        }
        return Eigen::Vector3<T>(
            arr.at(0).get<T>(),
            arr.at(1).get<T>(),
            arr.at(2).get<T>()
        );
    }


    static std::shared_ptr<AbstractObject<GLOBAL_CONFIG::PROJECT_TYPE>>
    createManpad(const nlohmann::json& device_config,
                 const std::shared_ptr<AbstractWorldModel<GLOBAL_CONFIG::PROJECT_TYPE>>& world) {

        using metricType = GLOBAL_CONFIG::PROJECT_TYPE;

        // 1. Загрузка конфигурации устройства
        const std::string config_path = device_config.at("config_path").get<std::string>();
        nlohmann::json device_params;
        {
            std::ifstream file(config_path);
            if (!file.is_open()) {
                throw std::runtime_error("Cannot open device config: " + config_path);
            }
            file >> device_params;
        }

        // 2. Создание менеджера интерполяции
        auto interp_mgr = std::make_shared<ComponentInterpolationManager<metricType>>();

        // 3. Загрузка тяги (1D интерполяция)
        if (device_params.contains("thrust_x_path") &&
            device_params.contains("thrust_y_path") &&
            device_params.contains("thrust_z_path")) {

            auto thrust_x = uploader1D_fromTXT<metricType>(
                device_params.at("thrust_x_path").get<std::string>()
            ).loadFromFile();

            auto thrust_y = uploader1D_fromTXT<metricType>(
                device_params.at("thrust_y_path").get<std::string>()
            ).loadFromFile();

            auto thrust_z = uploader1D_fromTXT<metricType>(
                device_params.at("thrust_z_path").get<std::string>()
            ).loadFromFile();

            interp_mgr->setThrust(
                std::move(thrust_x),
                std::move(thrust_y),
                std::move(thrust_z)
            );
        }

        // 4. Загрузка массы (1D интерполяция)
        if (device_params.contains("mass_path")) {
            auto mass_interp = uploader1D_fromTXT<metricType>(
                device_params.at("mass_path").get<std::string>()
            ).loadFromFile();
            interp_mgr->setMass(std::move(mass_interp));
        }

        // 5. Загрузка инерции (1D интерполяция)
        if (device_params.contains("Ixx_path") &&
            device_params.contains("Iyy_path") &&
            device_params.contains("Izz_path")) {

            auto Ixx = uploader1D_fromTXT<metricType>(
                device_params.at("Ixx_path").get<std::string>()
            ).loadFromFile();
            auto Iyy = uploader1D_fromTXT<metricType>(
                device_params.at("Iyy_path").get<std::string>()
            ).loadFromFile();
            auto Izz = uploader1D_fromTXT<metricType>(
                device_params.at("Izz_path").get<std::string>()
            ).loadFromFile();

            interp_mgr->setInertia(
                std::move(Ixx),
                std::move(Iyy),
                std::move(Izz)
            );
        }

        // 6. Загрузка центра масс (1D интерполяция)
        if (device_params.contains("COM_x_path") &&
            device_params.contains("COM_y_path") &&
            device_params.contains("COM_z_path")) {

            auto com_x = uploader1D_fromTXT<metricType>(
                device_params.at("COM_x_path").get<std::string>()
            ).loadFromFile();
            auto com_y = uploader1D_fromTXT<metricType>(
                device_params.at("COM_y_path").get<std::string>()
            ).loadFromFile();
            auto com_z = uploader1D_fromTXT<metricType>(
                device_params.at("COM_z_path").get<std::string>()
            ).loadFromFile();

            interp_mgr->setCOM(
                std::move(com_x),
                std::move(com_y),
                std::move(com_z)
            );
        }

        /*// 7. Загрузка аэродинамических коэффициентов (2D интерполяция, если есть)
        if (device_params.contains("cx_aero_path") &&
            device_params.contains("cy_aero_path") &&
            device_params.contains("cz_aero_path")) {

            auto cx = uploader2D_fromTXT<metricType>(
                device_params.at("cx_aero_path").get<std::string>()
            ).loadFromFile();
            auto cy = uploader2D_fromTXT<metricType>(
                device_params.at("cy_aero_path").get<std::string>()
            ).loadFromFile();
            auto cz = uploader2D_fromTXT<metricType>(
                device_params.at("cz_aero_path").get<std::string>()
            ).loadFromFile();

            interp_mgr->setAerodynamicForceCoefficients(
                std::move(cx),
                std::move(cy),
                std::move(cz)
            );
        }*/

        // 8. Создание провайдера параметров
        auto params_provider = std::make_shared<DynamicParametersProvider<metricType>>(
            ParameterProviderFactory<metricType>::createFromManager(interp_mgr)
        );

        // 9. Аэродинамическая модель
        std::shared_ptr<aero::AerodynamicsModel> aero_model;
        if (device_params.contains("aerodynamics_path")) {
            aero_model = aero::AerodynamicsModel::createFromFile(
                device_params.at("aerodynamics_path").get<std::string>()
            );
        } else {
            throw std::runtime_error("Aerodynamics model path not specified in device config");
        }

        // 10. Система ОДУ
        auto rocket_system = std::make_unique<FullRocketODE<metricType>>(
            params_provider,
            world,
            aero_model
        );

        // 11. Начальные условия
        auto init_params = std::make_unique<ObjInitParams<metricType>>();
        const auto& init_state = device_config.at("initial_state");

        init_params->position = parseVector3(init_state.at("position"));
        init_params->velocity = parseVector3(init_state.at("velocity"));
        init_params->eulerAngles = parseVector3(init_state.at("euler")) * MathConstants::PI / 180.0;
        init_params->angularVelocity = parseVector3(init_state.at("angular_velocity")) * MathConstants::PI / 180.0;


        // 12. Создание объекта
        auto manpad = std::make_shared<MANPAD_V1<metricType>>(
            std::move(rocket_system),
            std::move(init_params),
            interp_mgr
        );

        return manpad;
    }


    static std::shared_ptr<AbstractObject<GLOBAL_CONFIG::PROJECT_TYPE>>
createSimpleObject(const nlohmann::json& device_config,
                   const std::shared_ptr<AbstractWorldModel<GLOBAL_CONFIG::PROJECT_TYPE>>& /*world*/) {

        using metricType = GLOBAL_CONFIG::PROJECT_TYPE;

        // Читаем начальные условия напрямую из JSON
        const auto& init_state = device_config.at("initial_state");

        const auto position = parseVector3(init_state.at("position"));
        const auto velocity = parseVector3(init_state.at("velocity"));

        return std::make_shared<SimpleObject<metricType>>(
            position,
            velocity
        );
    }

    static std::shared_ptr<AbstractObject<GLOBAL_CONFIG::PROJECT_TYPE>>
createNetProjectile(const nlohmann::json& device_config,
                    const std::shared_ptr<AbstractWorldModel<GLOBAL_CONFIG::PROJECT_TYPE>>& world) {
        using metricType = GLOBAL_CONFIG::PROJECT_TYPE;

    // 1. Загружаем JSON конфиг устройства сеткомёта
    const std::string configpath = device_config.at("config_path").get<std::string>();
    nlohmann::json deviceparams;

    std::ifstream file(configpath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open net projectile config: " + configpath);
    }
    file >> deviceparams;

    // 2. Интерполятор массы m(t)
    auto interp_manager = std::make_shared<ComponentInterpolationManager<metricType>>();

    if (deviceparams.contains("mass_path")) {
        auto mass_interp = uploader1D_fromTXT<metricType>(
            deviceparams.at("mass_path").get<std::string>()).loadFromFile();
        interp_manager->setMass(std::move(mass_interp));
    } else {
        throw std::runtime_error("Net projectile config missing 'mass_path'");
    }

    // 3. Cx(alpha) до и после раскрытия + S_ref(t)
    if (!deviceparams.contains("cx_before_path") ||
        !deviceparams.contains("cx_after_path")  ||
        !deviceparams.contains("sref_path"))
    {
        throw std::runtime_error("Net projectile config must contain "
                                 "'cx_before_path', 'cx_after_path' and 'sref_path'");
    }

    // loader1D даёт векторы, из них собираем LinearInterpolator
    auto cx_before = uploader1D_fromTXT<metricType>(
        deviceparams.at("cx_before_path").get<std::string>()).loadFromFile();
    auto cx_after = uploader1D_fromTXT<metricType>(
        deviceparams.at("cx_after_path").get<std::string>()).loadFromFile();
    auto s_ref = uploader1D_fromTXT<metricType>(
        deviceparams.at("sref_path").get<std::string>()).loadFromFile();



    metricType deploy_start = deviceparams.value("deploy_start", metricType(1.5));
    metricType deploy_dur   = deviceparams.value("deploy_duration", metricType(0.2));

    auto aero_model = std::make_shared<NetAeroModel<metricType>>(
        std::move(cx_before),
        std::move(cx_after),
        std::move(s_ref),
        deploy_start,
        deploy_dur
    );

    // 4. Провайдер параметров (масса)
    auto param_provider_impl =
        ParameterProviderFactory<metricType>::createFromManager(interp_manager);
    auto params_provider =
        std::make_shared<DynamicParametersProvider<metricType>>(std::move(param_provider_impl));

    // 5. Динамическая система 3DOF
    auto dyn_sys = std::make_unique<NetProjectile3DOF<metricType>>(
        params_provider,
        world,
        aero_model
    );

    // 6. Начальные условия
    auto init_params = std::make_unique<ObjInitParams<metricType>>();
    const auto& initstate = device_config.at("initial_state");

    init_params->position = parseVector3(initstate.at("position"));
    init_params->velocity = parseVector3(initstate.at("velocity"));
        init_params->eulerAngles = parseVector3(initstate.at("euler")) * MathConstants::PI / 180.0;
        init_params->angularVelocity = parseVector3(initstate.at("angular_velocity")) * MathConstants::PI / 180.0;

    // 7. Создаём сам объект
    auto net = std::make_shared<NetProjectile<metricType>>(
        std::move(dyn_sys),
        std::move(init_params),
        interp_manager
    );

    return net;
    }


};

