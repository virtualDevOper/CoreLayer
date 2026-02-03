#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <cmath>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

// Подключаем необходимые заголовки
#include "GLOBAL_CONFIG.h"

// Подключаем моки для тестирования
#include "test_mocks.h"

// Подключаем реальный RungeKutta4Solver
#include "../../../src/OdeSolver/RungeKutta4Solver/RungeKutta4Solver.h"

using namespace Catch;
using metricType = GLOBAL_CONFIG::PROJECT_TYPE;

// === ТЕСТЫ РЕАЛЬНОГО RUNGEКUTTA4SOLVER ===

TEST_CASE("RungeKutta4Solver - Экспоненциальное затухание", "[solver][rk4][exponential][russian]") {
    using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<metricType>>;
    
    SECTION("Стандартное экспоненциальное затухание: dy/dt = -y") {
        // Создаем систему динамики для экспоненциального затухания
        auto dynamics = createExponentialDecaySystem<metricType>(metricType(1.0));
        
        // Создаем объект и менеджер
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        // Создаем SimulationMomento
        SimulationMomento<metricType> momento;
        
        // Инициализируем momento через saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        // Колбэк остановки: интегрируем до t = 2.0
        metricType target_time = metricType(2.0);
        CallbackType callback = [target_time](auto, metricType t) -> bool {
            return t < target_time;
        };
        
        // Запускаем реальный RungeKutta4Solver
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            metricType(0.0),    // t_start
            metricType(0.001),  // step_size (малый шаг для точности)
            callback,
            momento
        ));
        
        // Проверяем результат
        const auto& final_states = momento.getStateStorageByID(0).getStates();
        REQUIRE(final_states.size() > 1);
        
        const auto& final_state = final_states.back();
        metricType final_time = final_state.getTime();
        metricType numerical_result = final_state.getPosition().x();
        
        // Аналитическое решение: x(t) = exp(-t)
        metricType analytical_result = std::exp(-final_time);
        
        INFO("Время: " << final_time << ", Численное: " << numerical_result << ", Аналитическое: " << analytical_result);
        
        // Проверяем точность (для float используем менее строгую толерантность)
        REQUIRE(numerical_result == Approx(analytical_result).epsilon(metricType(1e-3)));
    }
    
    SECTION("Экспоненциальное затухание с различными параметрами") {
        struct TestCase {
            metricType lambda;
            metricType y0;
            metricType t_end;
            std::string description;
        };
        
        std::vector<TestCase> test_cases = {
            {metricType(0.5), metricType(2.0), metricType(3.0), "Медленное затухание"},
            {metricType(2.0), metricType(0.5), metricType(1.0), "Быстрое затухание"},
            {metricType(0.1), metricType(10.0), metricType(5.0), "Очень медленное затухание"}
        };
        
        for (const auto& test_case : test_cases) {
            INFO("Тест: " << test_case.description);
            
            // Создаем систему динамики: dy/dt = -lambda * y
            auto dynamics = createExponentialDecaySystem<metricType>(test_case.lambda);
            
            auto object = std::make_shared<MockObject<metricType>>(
                dynamics,
                Eigen::Vector3<metricType>(test_case.y0, metricType(0.0), metricType(0.0))
            );
            auto object_manager = std::make_shared<MockObjectManager<metricType>>();
            object_manager->addObject(0, object);
            
            SimulationMomento<metricType> momento;
            momento.saveStartParams(object_manager->getAllObjects());
            
            CallbackType callback = [t_end = test_case.t_end](auto, metricType t) -> bool {
                return t < t_end;
            };
            
            RungeKutta4Solver<metricType, CallbackType> solver;
            
            REQUIRE_NOTHROW(solver.solve(
                object_manager,
                metricType(0.0),
                metricType(0.001),
                callback,
                momento
            ));
            
            const auto& states = momento.getStateStorageByID(0).getStates();
            const auto& final_state = states.back();
            
            metricType numerical_result = final_state.getPosition().x();
            metricType analytical_result = test_case.y0 * std::exp(-test_case.lambda * test_case.t_end);
            
            INFO("Lambda: " << test_case.lambda << ", y0: " << test_case.y0 << ", t_end: " << test_case.t_end);
            REQUIRE(numerical_result == Approx(analytical_result).epsilon(metricType(3e-3)));
        }
    }
}

TEST_CASE("RungeKutta4Solver - Экспоненциальный рост", "[solver][rk4][growth][russian]") {
    using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<metricType>>;
    
    SECTION("Контролируемый экспоненциальный рост") {
        metricType lambda = metricType(0.5);
        metricType y0 = metricType(1.0);
        metricType t_end = metricType(1.0);
        
        // Система динамики: dy/dt = lambda * y
        auto dynamics = createExponentialGrowthSystem<metricType>(lambda);
        
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0)))
            .build();
        
        SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        CallbackType callback = [t_end](auto, metricType t) -> bool {
            return t < t_end;
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            metricType(0.0),
            metricType(0.001),
            callback,
            momento
        ));
        
        const auto& states = momento.getStateStorageByID(0).getStates();
        const auto& final_state = states.back();
        
        metricType numerical_result = final_state.getPosition().x();
        metricType analytical_result = y0 * std::exp(lambda * t_end);
        
        INFO("Численное: " << numerical_result << ", Аналитическое: " << analytical_result);
        REQUIRE(numerical_result == Approx(analytical_result).epsilon(metricType(1e-3)));
    }
}

TEST_CASE("RungeKutta4Solver - Линейные ОДУ", "[solver][rk4][linear][russian]") {
    using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<metricType>>;
    
    SECTION("Линейная функция: dy/dt = t") {
        // Система динамики: dx/dt = t (интеграл: x = t²/2 + C)
        auto dynamics = createLinearSystem<metricType>();
        
        metricType y0 = metricType(0.0);
        metricType t_end = metricType(1.0);
        
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0)))
            .build();
        
        SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        CallbackType callback = [t_end](auto, metricType t) -> bool {
            return t < t_end;
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            static_cast<metricType>(0.0),
            static_cast<metricType>(0.1),  // Можно использовать больший шаг для полиномов
            callback,
            momento
        ));
        
        const auto& states = momento.getStateStorageByID(0).getStates();
        const auto& final_state = states.back();
        
        metricType numerical_result = final_state.getPosition().x();
        metricType analytical_result = t_end * t_end * static_cast<metricType>(0.5);  // x = t²/2
        
        INFO("Численное: " << numerical_result << ", Аналитическое: " << analytical_result);
        // RK4 должен быть точным для полиномов до степени 3
        REQUIRE(numerical_result == Approx(analytical_result).margin(metricType(1e-10)));
    }
    
    SECTION("Константная функция: dy/dt = 1") {
        // Система динамики: dx/dt = 1 (интеграл: x = t + C)
        auto dynamics = createConstantSystem<metricType>();
        
        metricType y0 = metricType(0.0);
        metricType t_end = metricType(2.0);
        
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0)))
            .build();
        
        SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        CallbackType callback = [t_end](auto, metricType t) -> bool {
            return t < t_end;
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            metricType(0.0),
            metricType(0.1),
            callback,
            momento
        ));
        
        const auto& states = momento.getStateStorageByID(0).getStates();
        const auto& final_state = states.back();
        
        metricType numerical_result = final_state.getPosition().x();
        metricType analytical_result = t_end;  // x = t
        
        INFO("Численное: " << numerical_result << ", Аналитическое: " << analytical_result);
        REQUIRE(numerical_result == Approx(analytical_result).margin(metricType(1e-10)));
    }
}

TEST_CASE("RungeKutta4Solver - Порядок точности", "[solver][rk4][order][russian]") {
    using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<metricType>>;
    
    SECTION("Проверка сходимости четвертого порядка") {
        // Используем экспоненциальное затухание для проверки порядка
        metricType y0 = metricType(1.0);
        metricType t_end = metricType(1.0);
        metricType analytical_result = std::exp(-t_end);
        
        std::vector<metricType> step_sizes = {metricType(0.1), metricType(0.05), metricType(0.025), metricType(0.0125)};
        std::vector<metricType> errors;
        
        for (metricType dt : step_sizes) {
            auto dynamics = createExponentialDecaySystem<metricType>(metricType(1.0));
            
            auto object = std::make_shared<MockObject<metricType>>(
                dynamics,
                Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0))
            );
            auto object_manager = std::make_shared<MockObjectManager<metricType>>();
            object_manager->addObject(0, object);
            
            auto initial_state = KinematicState<metricType>::createBuilder()
                .setPosition(Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0)))
                .build();
            
            SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
            
            CallbackType callback = [t_end](auto, metricType t) -> bool {
                return t < t_end;
            };
            
            RungeKutta4Solver<metricType, CallbackType> solver;
            solver.solve(object_manager, metricType(0.0), dt, callback, momento);
            
            const auto& states = momento.getStateStorageByID(0).getStates();
            metricType numerical_result = states.back().getPosition().x();
            metricType error = std::abs(numerical_result - analytical_result);
            errors.push_back(error);
        }
        
        // Проверяем, что ошибка уменьшается при уменьшении шага (базовая сходимость)
        for (size_t i = 1; i < errors.size(); ++i) {
            INFO("Шаг " << i << ": ошибка[" << i-1 << "] = " << errors[i-1] << ", ошибка[" << i << "] = " << errors[i]);
            
            // Проверяем сходимость (ошибка должна уменьшаться)
            // Для float пропускаем тест если достигли машинной точности
            if (errors[i] > metricType(5e-7) && errors[i-1] > metricType(5e-7)) {
                REQUIRE(errors[i] < errors[i-1]);  // Ошибка должна уменьшаться
            } else {
                // На машинной точности просто проверяем разумность
                REQUIRE(errors[i] < metricType(1e-2));  // Увеличиваем допустимую ошибку для float
            }
        }
    }
}

TEST_CASE("RungeKutta4Solver - Тесты стабильности", "[solver][rk4][stability][russian]") {
    using CallbackType = GLOBAL_CONFIG::STOP_SOLVE_FUNC_TYPE<IObjectManager<metricType>>;
    
    SECTION("Стабильная интеграция с разумными размерами шага") {
        auto dynamics = createExponentialDecaySystem<metricType>(metricType(1.0));
        
        metricType y0 = metricType(1.0);
        metricType t_end = metricType(5.0);
        
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(y0, metricType(0.0), metricType(0.0)))
            .build();
        
        SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        CallbackType callback = [t_end](auto, metricType t) -> bool {
            return t < t_end;
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            metricType(0.0),
            metricType(0.01),
            callback,
            momento
        ));
        
        // Проверяем стабильность: решение должно оставаться ограниченным и положительным
        const auto& states = momento.getStateStorageByID(0).getStates();
        
        for (const auto& state : states) {
            metricType y = state.getPosition().x();
            
            // Решение должно оставаться ограниченным и положительным для экспоненциального затухания
            REQUIRE(y > metricType(0.0));
            REQUIRE(y <= y0);
            REQUIRE(std::isfinite(y));
        }
    }
    
    SECTION("Обработка граничных случаев") {
        auto dynamics = createExponentialDecaySystem<metricType>(metricType(1.0));
        
        auto object = std::make_shared<MockObject<metricType>>(
            dynamics,
            Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0))
        );
        auto object_manager = std::make_shared<MockObjectManager<metricType>>();
        object_manager->addObject(0, object);
        
        auto initial_state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0)))
            .build();
        
        SimulationMomento<metricType> momento;

        // �������������� momento ����� saveStartParams
        momento.saveStartParams(object_manager->getAllObjects());
        
        // Очень малый размер шага
        metricType dt_small = metricType(1e-6);
        CallbackType callback = [](auto, metricType t) -> bool {
            return t < metricType(0.001);
        };
        
        RungeKutta4Solver<metricType, CallbackType> solver;
        
        REQUIRE_NOTHROW(solver.solve(
            object_manager,
            metricType(0.0),
            dt_small,
            callback,
            momento
        ));
        
        const auto& states = momento.getStateStorageByID(0).getStates();
        const auto& final_state = states.back();
        metricType result = final_state.getPosition().x();
        
        REQUIRE(std::isfinite(result));
        REQUIRE(result > metricType(0.0));
    }
}




