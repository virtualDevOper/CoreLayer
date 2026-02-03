#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <memory>
#include <map>

// Тестируем ваш реальный RungeKutta4Solver
#include "../../../src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"
#include "../../../src/DynamicsSystem/IDynamicsSystem.h"
#include "../../../src/utils/ObjManager/IObjectManager.h"
#include "../../../src/SimulationMomento/SimulationMomento.h"
#include "../../../src/PhysicalObjects/AbstractObject.h"

// Подключаем моки
#include "test_mocks.h"

using namespace Catch;
using metricType = float;

// === МОКИ ДЛЯ ТЕСТИРОВАНИЯ ===
// Используем моки из test_mocks.h

// === ТЕСТЫ ВАШЕГО РЕАЛЬНОГО RUNGEКUTTA4SOLVER ===

TEST_CASE("RungeKutta4Solver базовая функциональность", "[solver][rk4][integration]") {
    using CallbackType = std::function<bool(std::shared_ptr<IObjectManager<metricType>>, metricType)>;
    
    SECTION("Решатель интегрирует экспоненциальное затухание") {
        // Создаем мок системы динамики для y' = -y
        auto dynamics = std::make_shared<MockDynamicsSystem<metricType>>(
            [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
                // Производная: позиция' = -позиция (экспоненциальное затухание)
                return KinematicState<metricType>::createBuilder()
                    .setPosition(-state.getPosition())
                    .setVelocity(Eigen::Vector3<metricType>::Zero())
                    .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                    .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                    .build();
            }
        );
        
        // Создаем мок объекта
        auto object = std::make_shared<MockObject<metricType>>(dynamics);
        
        // Создаем мок менеджера объектов
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        // Создаем начальное состояние
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 0.0f, 0.0f))
            .build();
        
        // Создаем SimulationMomento и добавляем начальное состояние
        SimulationMomento<metricType> momento;
        
        // Инициализируем momento через saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        // Создаем колбэк остановки (остановиться после нескольких шагов)
        int step_count = 0;
        CallbackType callback = [&step_count](auto, metricType t) -> bool {
            step_count++;
            return step_count < 100 && t < 1.0f;  // Остановиться через 100 шагов или при t >= 1.0
        };
        
        // Создаем и запускаем решатель
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            0.0f,      // t_start
            0.01f,     // step_size
            callback,
            momento
        ));
        
        // Проверяем результат
        const auto& final_states = momento.getStateStorageByID(0).getStates();
        REQUIRE(final_states.size() > 1);  // Должно быть больше одного состояния
        
        const auto& final_state = final_states.back();
        metricType final_time = final_state.getTime();
        metricType final_position = final_state.getPosition().x();
        
        // Аналитическое решение: y(t) = exp(-t)
        metricType analytical = std::exp(-final_time);
        
        INFO("Время: " << final_time << ", Численное: " << final_position << ", Аналитическое: " << analytical);
        REQUIRE(final_position == Approx(analytical).margin(1e-3f));
    }
}

TEST_CASE("RungeKutta4Solver обработка ошибок", "[solver][rk4][error]") {
    using CallbackType = std::function<bool(std::shared_ptr<IObjectManager<metricType>>, metricType)>;
    
    SECTION("Отрицательный шаг времени вызывает исключение") {
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        SimulationMomento<metricType> momento;
        CallbackType callback = [](auto, auto) { return false; };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_THROWS_AS(solver.solve(
            object_manager,
            0.0f,
            -0.01f,  // Отрицательный шаг
            callback,
            momento
        ), std::invalid_argument);
    }
    
    SECTION("Нулевой шаг времени вызывает исключение") {
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        SimulationMomento<metricType> momento;
        CallbackType callback = [](auto, auto) { return false; };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_THROWS_AS(solver.solve(
            object_manager,
            0.0f,
            0.0f,  // Нулевой шаг
            callback,
            momento
        ), std::invalid_argument);
    }
    
    SECTION("Отрицательное начальное время вызывает исключение") {
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        SimulationMomento<metricType> momento;
        CallbackType callback = [](auto, auto) { return false; };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_THROWS_AS(solver.solve(
            object_manager,
            -1.0f,  // Отрицательное время
            0.01f,
            callback,
            momento
        ), std::invalid_argument);
    }
    
    SECTION("Пустой менеджер объектов вызывает исключение") {
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();  // Пустой
        SimulationMomento<metricType> momento;
        CallbackType callback = [](auto, auto) { return false; };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_THROWS_AS(solver.solve(
            object_manager,
            0.0f,
            0.01f,
            callback,
            momento
        ), std::invalid_argument);
    }
}

TEST_CASE("RungeKutta4Solver множественные объекты", "[solver][rk4][multiple]") {
    using CallbackType = std::function<bool(std::shared_ptr<IObjectManager<metricType>>, metricType)>;
    
    SECTION("Решатель обрабатывает несколько объектов одновременно") {
        // Создаем две разные системы динамики
        auto dynamics1 = std::make_shared<MockDynamicsSystem<metricType>>(
            [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
                // Объект 1: y' = -y (затухание)
                return KinematicState<metricType>::createBuilder()
                    .setPosition(-state.getPosition())
                    .build();
            }
        );
        
        auto dynamics2 = std::make_shared<MockDynamicsSystem<metricType>>(
            [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
                // Объект 2: y' = 0.5*y (рост)
                return KinematicState<metricType>::createBuilder()
                    .setPosition(0.5f * state.getPosition())
                    .build();
            }
        );
        
        // Создаем объекты с разными начальными условиями
        auto object1 = std::make_shared<MockObject<metricType>>(
            dynamics1, 
            Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0))
        );
        auto object2 = std::make_shared<MockObject<metricType>>(
            dynamics2,
            Eigen::Vector3<metricType>(metricType(2.0), metricType(0.0), metricType(0.0))
        );
        
        // Создаем менеджер объектов
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object1);
        object_manager->addObject(1, object2);
        
        // Создаем SimulationMomento
        SimulationMomento<metricType> momento;
        
        // Инициализируем momento через saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        // Колбэк остановки
        int step_count = 0;
        CallbackType callback = [&step_count](auto, metricType t) -> bool {
            step_count++;
            return step_count < 50 && t < 0.5f;
        };
        
        // Запускаем решатель
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            0.0f,
            0.01f,
            callback,
            momento
        ));
        
        // Проверяем результаты для обоих объектов
        const auto& states1 = momento.getStateStorageByID(0).getStates();
        const auto& states2 = momento.getStateStorageByID(1).getStates();
        
        REQUIRE(states1.size() > 1);
        REQUIRE(states2.size() > 1);
        REQUIRE(states1.size() == states2.size());  // Одинаковое количество шагов
        
        const auto& final1 = states1.back();
        const auto& final2 = states2.back();
        
        metricType t_final = final1.getTime();
        
        // Проверяем аналитические решения
        metricType analytical1 = 1.0f * std::exp(-t_final);      // y1' = -y1, y1(0) = 1
        metricType analytical2 = 2.0f * std::exp(0.5f * t_final); // y2' = 0.5*y2, y2(0) = 2
        
        INFO("Время: " << t_final);
        INFO("Объект 1 - Численное: " << final1.getPosition().x() << ", Аналитическое: " << analytical1);
        INFO("Объект 2 - Численное: " << final2.getPosition().x() << ", Аналитическое: " << analytical2);
        
        REQUIRE(final1.getPosition().x() == Approx(analytical1).margin(1e-3f));
        REQUIRE(final2.getPosition().x() == Approx(analytical2).margin(1e-3f));
    }
}

TEST_CASE("RungeKutta4Solver неактивные объекты", "[solver][rk4][inactive]") {
    using CallbackType = std::function<bool(std::shared_ptr<IObjectManager<metricType>>, metricType)>;
    
    SECTION("Неактивные объекты не интегрируются") {
        auto dynamics = std::make_shared<MockDynamicsSystem<metricType>>(
            [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
                // Эта функция не должна вызываться для неактивного объекта
                FAIL("Система динамики вызвана для неактивного объекта!");
                return state;
            }
        );
        
        // Создаем неактивный объект
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0)),
            false  // false = неактивный
        );
        
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        SimulationMomento<metricType> momento;
        
        // Инициализируем momento через saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        int step_count = 0;
        CallbackType callback = [&step_count](auto, auto) -> bool {
            step_count++;
            return step_count < 10;  // Несколько шагов
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            0.0f,
            0.01f,
            callback,
            momento
        ));
        
        // Проверяем, что состояние не изменилось
        const auto& states = momento.getStateStorageByID(0).getStates();
        REQUIRE(states.size() > 1);  // Должны быть добавлены копии состояния
        
        // Все состояния должны быть одинаковыми (объект неактивен)
        for (const auto& state : states) {
            REQUIRE(state.getPosition().x() == Approx(1.0f).margin(1e-6f));
        }
    }
}
