cmake_minimum_required(VERSION 3.30)
project(CoreLayer)

set(CMAKE_CXX_STANDARD 23)

# Add cmake modules directory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Добавляем директории для поиска заголовочных файлов
include_directories(${CMAKE_SOURCE_DIR}/include/core)
include_directories(${CMAKE_SOURCE_DIR}/libs/eigen-3.4.0)
include_directories(${CMAKE_SOURCE_DIR}/libs)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)



add_executable(CoreLayer main.cpp
        include/core/PCH.h
        include/core/Model.h
        include/core/Model.cpp
        include/core/CONSTANTS.h
        include/core/GLOBAL_CONFIG.h
        src/OdeSolver/ODESolver.h
        src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h
        src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.cpp
        src/DynamicsSystem/IDynamicsSystem.h
        src/DynamicsSystem/FullRocketODE/FullRocketODE.h
        src/DynamicsSystem/FullRocketODE/FullRocketODE.cpp
        src/WorldModel/AbstractWorldModel.h
        src/WorldModel/WorldComponents/WindModel/IWindModel.h
        src/WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.cpp
        src/WorldModel/WorldComponents/WindModel/NoWindModel/NoWindModel.h
        src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.cpp
        src/WorldModel/ConcreteWorld/SimpleWorld/SimpleWorld.h
        src/WorldModel/WorldComponents/AtmosphericModel/IAtmosphericModel.h
        src/WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.cpp
        src/WorldModel/WorldComponents/AtmosphericModel/SimpleAtmosphericModel/AtmosphericModel.h
        src/WorldModel/WorldComponents/GravityModel/IGravitationalModel.h
        src/WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.cpp
        src/WorldModel/WorldComponents/GravityModel/KavendishModel/KavendishModel.h
        src/WorldModel/WorldComponents/CoriolisModel/ICoriolisModel.h
        src/WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.cpp
        src/WorldModel/WorldComponents/CoriolisModel/NoCoriolisForceModel/NoCoriolisForceModel.h
        src/WorldModel/WorldComponents/ITerrainModel/ITerrainModel.h
        src/WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.cpp
        src/WorldModel/WorldComponents/ITerrainModel/PlaneTerrain/PlaneTerrain.h
        src/PhysicalObjects/Aircraft/AbstractAircraft.h
        src/SimulationMomento/SimulationMomento.h
        src/utils/ObjSnapshot.h
        src/utils/ObjSnapshot.cpp
        src/SimulationMomento/SaveStrategy/SaveStrategy.h
        src/SimulationMomento/StateStorage.h
        src/PhysicalObjects/AbstractObject.h
        src/PhysicalObjects/AbstractObject.cpp
        src/PhysicalObjects/Aircraft/Missle/GuidedMissle/MANPADS/V1/MANPAD_V1.h
        src/utils/ObjInitParams.h
        src/utils/ObjManager/ObjectManager.h
        src/utils/SimulationDescriber.h
        src/utils/SimulationDescriber.cpp
        src/utils/ContinuationCallback.h
        src/utils/Interpolation/ILinearInterpolator/ILinearInterpolator.h
        src/utils/Interpolation/IBilinearInterpolator/IBilinearInterpolator.h
        src/utils/Interpolation/ILinearInterpolator/LinearInterpolator/LinearInterpolator.cpp
        src/utils/Interpolation/ILinearInterpolator/LinearInterpolator/LinearInterpolator.h
        src/utils/Interpolation/IBilinearInterpolator/BilinearInterpolator/BilinearInterpolator.cpp
        src/utils/Interpolation/IBilinearInterpolator/BilinearInterpolator/BilinearInterpolator.h
        src/utils/DataTable/DataTable1D/uploader1D_fromTXT.cpp
        src/utils/DataTable/DataTable1D/uploader1D_fromTXT.h
        src/utils/DataTable/IDataUploader.h
        src/utils/DataTable/DataTable2D/uploader2D_fromTXT.cpp
        src/utils/DataTable/DataTable2D/uploader2D_fromTXT.h
        src/PhysicalObjects/Aircraft/Missle/GuidedMissle/GuidedMissle.h
        src/DynamicsSystem/SimpleKinematicsSystem/SimpleKinematicsSystem.cpp
        src/DynamicsSystem/SimpleKinematicsSystem/SimpleKinematicsSystem.h
        src/PhysicalObjects/SimpleObject/SimpleObject.cpp
        src/PhysicalObjects/SimpleObject/SimpleObject.h
        src/utils/Interpolation/ComponentInterpolationManager.h
        src/utils/DynamicParametersProviderForFullRocketModel.h
        src/utils/ObjManager/IObjectManager.h

        src/utils/KinematicState/KinematicState.h
        src/utils/KinematicState/KinematicState.cpp
        src/utils/SimulationConfig.cpp
        src/utils/SimulationConfig.h
        src/utils/IParameterProvider.h
        src/utils/KinematicStateDerivative/KinematicStateDerivative.cpp
        src/utils/KinematicStateDerivative/KinematicStateDerivative.h
        
        # Aerodynamics library sources
        include/aero_simpi/src/component.cpp
        include/aero_simpi/src/model.cpp
        include/aero_simpi/src/parser.cpp
)



