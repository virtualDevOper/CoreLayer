#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>

#include "../../../src/utils/KinematicState.h"

using namespace Catch;

TEST_CASE("KinematicState Builder паттерн работает корректно", "[math][kinematic][builder]") {
    using metricType = float;
    
    SECTION("Конструктор по умолчанию создает нулевое состояние") {
        auto state = KinematicState<metricType>::createBuilder().build();
        
        REQUIRE(state.getPosition().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(state.getVelocity().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(state.getEulerAngles().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(state.getAngularVelocity().norm() == Approx(0.0f).margin(1e-6f));
    }
    
    SECTION("Builder устанавливает значения корректно") {
        Eigen::Vector3<metricType> pos(1.0f, 2.0f, 3.0f);
        Eigen::Vector3<metricType> vel(4.0f, 5.0f, 6.0f);
        Eigen::Vector3<metricType> angles(0.1f, 0.2f, 0.3f);
        Eigen::Vector3<metricType> angVel(0.4f, 0.5f, 0.6f);
        
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(pos)
            .setVelocity(vel)
            .setEulerAngles(angles)
            .setAngularVelocity(angVel)
            .build();
        
        REQUIRE(state.getPosition().isApprox(pos, 1e-6f));
        REQUIRE(state.getVelocity().isApprox(vel, 1e-6f));
        REQUIRE(state.getEulerAngles().isApprox(angles, 1e-6f));
        REQUIRE(state.getAngularVelocity().isApprox(angVel, 1e-6f));
    }
}

TEST_CASE("KinematicState арифметические операции для RK4 интеграции", "[math][kinematic][rk4][integration]") {
    using metricType = float;
    
    SECTION("Оператор сложения работает корректно") {
        auto state1 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .setEulerAngles(Eigen::Vector3<metricType>(0.01f, 0.02f, 0.03f))
            .setAngularVelocity(Eigen::Vector3<metricType>(0.001f, 0.002f, 0.003f))
            .build();
            
        auto state2 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(4.0f, 5.0f, 6.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.4f, 0.5f, 0.6f))
            .setEulerAngles(Eigen::Vector3<metricType>(0.04f, 0.05f, 0.06f))
            .setAngularVelocity(Eigen::Vector3<metricType>(0.004f, 0.005f, 0.006f))
            .build();
        
        auto result = state1 + state2;
        
        Eigen::Vector3<metricType> expectedPos(5.0f, 7.0f, 9.0f);
        Eigen::Vector3<metricType> expectedVel(0.5f, 0.7f, 0.9f);
        Eigen::Vector3<metricType> expectedAngles(0.05f, 0.07f, 0.09f);
        Eigen::Vector3<metricType> expectedAngVel(0.005f, 0.007f, 0.009f);
        
        REQUIRE(result.getPosition().isApprox(expectedPos, 1e-6f));
        REQUIRE(result.getVelocity().isApprox(expectedVel, 1e-6f));
        REQUIRE(result.getEulerAngles().isApprox(expectedAngles, 1e-6f));
        REQUIRE(result.getAngularVelocity().isApprox(expectedAngVel, 1e-6f));
    }
    
    SECTION("Скалярное умножение работает корректно") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(2.0f, 4.0f, 6.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.2f, 0.4f, 0.6f))
            .setEulerAngles(Eigen::Vector3<metricType>(0.02f, 0.04f, 0.06f))
            .setAngularVelocity(Eigen::Vector3<metricType>(0.002f, 0.004f, 0.006f))
            .build();
        
        metricType scalar = 0.5f;
        auto result = state * scalar;
        
        Eigen::Vector3<metricType> expectedPos(1.0f, 2.0f, 3.0f);
        Eigen::Vector3<metricType> expectedVel(0.1f, 0.2f, 0.3f);
        Eigen::Vector3<metricType> expectedAngles(0.01f, 0.02f, 0.03f);
        Eigen::Vector3<metricType> expectedAngVel(0.001f, 0.002f, 0.003f);
        
        REQUIRE(result.getPosition().isApprox(expectedPos, 1e-6f));
        REQUIRE(result.getVelocity().isApprox(expectedVel, 1e-6f));
        REQUIRE(result.getEulerAngles().isApprox(expectedAngles, 1e-6f));
        REQUIRE(result.getAngularVelocity().isApprox(expectedAngVel, 1e-6f));
    }
    
    SECTION("Глобальный оператор скалярного умножения работает") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
        
        metricType scalar = 2.0f;
        auto result = scalar * state;
        
        Eigen::Vector3<metricType> expectedPos(2.0f, 4.0f, 6.0f);
        Eigen::Vector3<metricType> expectedVel(0.2f, 0.4f, 0.6f);
        
        REQUIRE(result.getPosition().isApprox(expectedPos, 1e-6f));
        REQUIRE(result.getVelocity().isApprox(expectedVel, 1e-6f));
    }
}

TEST_CASE("KinematicState математические свойства RK4 интеграции", "[math][kinematic][rk4][properties]") {
    using metricType = float;
    
    SECTION("Сложение коммутативно") {
        auto state1 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
            
        auto state2 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(4.0f, 5.0f, 6.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.4f, 0.5f, 0.6f))
            .build();
        
        auto result1 = state1 + state2;
        auto result2 = state2 + state1;
        
        REQUIRE(result1.getPosition().isApprox(result2.getPosition(), 1e-6f));
        REQUIRE(result1.getVelocity().isApprox(result2.getVelocity(), 1e-6f));
        REQUIRE(result1.getEulerAngles().isApprox(result2.getEulerAngles(), 1e-6f));
        REQUIRE(result1.getAngularVelocity().isApprox(result2.getAngularVelocity(), 1e-6f));
    }
    
    SECTION("Скалярное умножение ассоциативно") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
        
        metricType a = 2.0f;
        metricType b = 3.0f;
        
        auto result1 = (a * b) * state;
        auto result2 = a * (b * state);
        
        REQUIRE(result1.getPosition().isApprox(result2.getPosition(), 1e-6f));
        REQUIRE(result1.getVelocity().isApprox(result2.getVelocity(), 1e-6f));
    }
    
    SECTION("Дистрибутивное свойство выполняется") {
        auto state1 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
            
        auto state2 = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(4.0f, 5.0f, 6.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.4f, 0.5f, 0.6f))
            .build();
        
        metricType scalar = 2.0f;
        
        auto result1 = scalar * (state1 + state2);
        auto result2 = (scalar * state1) + (scalar * state2);
        
        REQUIRE(result1.getPosition().isApprox(result2.getPosition(), 1e-6f));
        REQUIRE(result1.getVelocity().isApprox(result2.getVelocity(), 1e-6f));
        REQUIRE(result1.getEulerAngles().isApprox(result2.getEulerAngles(), 1e-6f));
        REQUIRE(result1.getAngularVelocity().isApprox(result2.getAngularVelocity(), 1e-6f));
    }
}

TEST_CASE("KinematicState граничные случаи и стабильность", "[math][kinematic][stability][edge]") {
    using metricType = float;
    
    SECTION("Умножение на нулевой скаляр") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
        
        auto result = metricType(0.0f) * state;
        
        REQUIRE(result.getPosition().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(result.getVelocity().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(result.getEulerAngles().norm() == Approx(0.0f).margin(1e-6f));
        REQUIRE(result.getAngularVelocity().norm() == Approx(0.0f).margin(1e-6f));
    }
    
    SECTION("Умножение на единичный скаляр") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1.0f, 2.0f, 3.0f))
            .setVelocity(Eigen::Vector3<metricType>(0.1f, 0.2f, 0.3f))
            .build();
        
        auto result = metricType(1.0f) * state;
        
        REQUIRE(result.getPosition().isApprox(state.getPosition(), 1e-6f));
        REQUIRE(result.getVelocity().isApprox(state.getVelocity(), 1e-6f));
        REQUIRE(result.getEulerAngles().isApprox(state.getEulerAngles(), 1e-6f));
        REQUIRE(result.getAngularVelocity().isApprox(state.getAngularVelocity(), 1e-6f));
    }
    
    SECTION("Стабильность с большими значениями") {
        auto state = KinematicState<metricType>::createBuilder()
            .setPosition(Eigen::Vector3<metricType>(1e6f, 2e6f, 3e6f))
            .setVelocity(Eigen::Vector3<metricType>(1e3f, 2e3f, 3e3f))
            .build();
        
        auto result = state * metricType(1e-3f);
        
        Eigen::Vector3<metricType> expectedPos(1e3f, 2e3f, 3e3f);
        Eigen::Vector3<metricType> expectedVel(1.0f, 2.0f, 3.0f);
        
        REQUIRE(result.getPosition().isApprox(expectedPos, 1e-3f));
        REQUIRE(result.getVelocity().isApprox(expectedVel, 1e-6f));
    }
}