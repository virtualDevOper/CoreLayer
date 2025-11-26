
#include "include/publicCore/Model.h"
#include "src/DynamicsSystem/FullRocketODE/FullRocketODE.h"
#include "src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"
#include "src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.h"

#include "src//WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.h"
#include "src//WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.h"
#include "src/WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.h"
#include "src/WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.h"
#include "src/WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.h"
#include "src/PhysicalObjects/Aircraft/AbstractAircraft.h"
#include "src/PhysicalObjects/Target/Target.h"
#include "src/PhysicalObjects/ObjectManager.h"
#include "PCH.h"
#include "src/PhysicalObjects/Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h"
#include "src/DynamicsSystem/ExtensionModels/Aerodinamics/AeroInput/AeroInput.h"
#include "src/SimulationMomento/SimulationMomento.h"
#include "src/utils/SimulationDescriber.h"
#include "src/utils/ContinuationCallback.h"
#include "src/utils/DataTable/DataTable1D/uploader1D_fromTXT.h"
#include "src/utils/DataTable/DataTable2D/uploader2D_fromTXT.h"


//TODO
// == ИСПРАВИТЬ ТИПЫ УКАЗАТЕЛЕЙ НА SHARED
// == МАКСИМАЛЬНО БЫСТРО ПОПЫТАТЬСЯ ЗАПУСТИТЬ ПРИЛОЖЕНИЕ И ПОТОМ НАРАЩИВАТЬ МЯСО, А ТО ТАК НИКОГДА НЕ ПОЛУЧИСТЯ

int main() {
    /*
    auto solver = std::make_shared<RungeKutta4Solver<GLOBAL_CONFIG::PROJECT_TYPE, GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE>>();

    auto manager = std::make_shared<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>>();



    auto golubka_V1_ID = manager->addTrackedObject(golubka_V1);

    auto continue_callback = [&golubka_V1_ID,&manager](
        const std::shared_ptr<ObjectManager<GLOBAL_CONFIG::PROJECT_TYPE>> &) -> bool {
        return isMainRocKetActive(manager, golubka_V1_ID); // используем переданные аргументы
    };

    std::vector<int> mainTrackableObjects_IDs;
    mainTrackableObjects_IDs.push_back(golubka_V1_ID);





    IModel<GLOBAL_CONFIG::PROJECT_TYPE, GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE> model(
         solver,
         world,
         csvDataSaver,
         describer,
         manager,
         continue_callback,
         0.01
     );

    model.run();
    */
    const auto world = std::make_unique<SimpleWorld<GLOBAL_CONFIG::PROJECT_TYPE>>();
    {
        auto wind = std::make_unique<NoWindModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto atmospher = std::make_unique<AtmosphericModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto gravity = std::make_unique<KavendishModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto cariolis = std::make_unique<NoCoriolisForceModel<GLOBAL_CONFIG::PROJECT_TYPE>>();
        auto terrain = std::make_unique<PlaneTerrain<GLOBAL_CONFIG::PROJECT_TYPE>>();
        world->setAtmosphericModel(std::move(atmospher));
        world->setWindModel(std::move(wind));
        world->setGravityModel(std::move(gravity));
        world->setCoriolisEffect(std::move(cariolis));
        world->setTerrain(std::move(terrain));
    } // инициализация мира
    auto describer = std::make_unique<SimulationDescriber>(3);

    /*auto csvDataSaver = std::make_shared<SimulationMomento<GLOBAL_CONFIG::PROJECT_TYPE>>();
    csvDataSaver->setStrategy(std::make_shared<CsvSaveStrategy<GLOBAL_CONFIG::PROJECT_TYPE>>("simulation_data.csv"));
    std::vector<StateStorage<GLOBAL_CONFIG::PROJECT_TYPE>> trackedObjects;
    csvDataSaver->addTrackedObjs(std::move(trackedObjects));*/

    auto Fx_t_data = uploader1D_fromTXT<GLOBAL_CONFIG::PROJECT_TYPE>("dataTables/golubka_V1/Fx_t.txt");
    auto data  = Fx_t_data.loadFromFile();
    auto e = data->interpolate(1);
    /*проверить биленейную интерполяцию*/
    auto Cx_data = uploader2D_fromTXT<GLOBAL_CONFIG::PROJECT_TYPE>("dataTables/golubka_V1/Cx_a_m.txt");
    auto i = Cx_data.loadFromFile();
    auto t = i->interpolate(0.8, 0);
    auto golubka_V1_system = std::make_unique<FullRocketODE<GLOBAL_CONFIG::PROJECT_TYPE>>();
    /*auto golubka_V1_init_params = std::make_shared<ObjInitParams<GLOBAL_CONFIG::PROJECT_TYPE>>();
    auto golubka_V1_aero_input = std::make_shared<AeroInput<GLOBAL_CONFIG::PROJECT_TYPE>>();

    auto golubka_V1 = std::make_shared<MANPAD_V1<GLOBAL_CONFIG::PROJECT_TYPE>>(
        std::move(golubka_V1_system),
        std::move(golubka_V1_init_params),
        std::move(golubka_V1_aero_input)
     );*/
    return 0;
}

// TIP See CLion help at <a
// href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>.
//  Also, you can try interactive lessons for CLion by selecting
//  'Help | Learn IDE Features' from the main menu.