#define _USE_MATH_DEFINES
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>

#include "../../../src/utils/ConeDirection/ConeDirection.h"
#include <cmath>

using namespace Catch;

TEST_CASE("ConeDirection трансформации координат", "[math][transformation][matrix]") {
    using metricType = float;
    
    SECTION("Тождественная трансформация (нулевые углы)") {
        metricType theta = 0.0f;  // тангаж
        metricType psi = 0.0f;    // рыскание  
        metricType gamma = 0.0f;  // крен
        
        std::array<metricType, 3> vector = {1.0f, 0.0f, 0.0f};
        
        auto bodyToEarth = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, vector);
        auto earthToBody = TransformationFactory<metricType>::createEarthToBodyTransform(
            theta, psi, gamma, vector);
        
        auto resultBE = bodyToEarth->result_getter();
        auto resultEB = earthToBody->result_getter();
        
        // При нулевых углах трансформации должны быть идентичными
        REQUIRE(resultBE[0] == Approx(1.0f).margin(1e-6f));
        REQUIRE(resultBE[1] == Approx(0.0f).margin(1e-6f));
        REQUIRE(resultBE[2] == Approx(0.0f).margin(1e-6f));
        
        REQUIRE(resultEB[0] == Approx(1.0f).margin(1e-6f));
        REQUIRE(resultEB[1] == Approx(0.0f).margin(1e-6f));
        REQUIRE(resultEB[2] == Approx(0.0f).margin(1e-6f));
    }
    
    SECTION("Чистое вращение по тангажу (только theta)") {
        metricType theta = static_cast<metricType>(M_PI / 4.0);  // 45 градусов
        metricType psi = 0.0f;
        metricType gamma = 0.0f;
        
        std::array<metricType, 3> vector = {1.0f, 0.0f, 0.0f};
        
        auto bodyToEarth = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, vector);
        
        auto result = bodyToEarth->result_getter();
        
        metricType cos45 = std::cos(static_cast<metricType>(M_PI / 4.0));
        metricType sin45 = std::sin(static_cast<metricType>(M_PI / 4.0));
        
        REQUIRE(result[0] == Approx(cos45).margin(1e-6f));
        REQUIRE(result[1] == Approx(sin45).margin(1e-6f));
        REQUIRE(result[2] == Approx(0.0f).margin(1e-6f));
    }
    
    SECTION("Чистое вращение по рысканию (только psi)") {
        metricType theta = 0.0f;
        metricType psi = static_cast<metricType>(M_PI / 4.0);  // 45 градусов
        metricType gamma = 0.0f;
        
        std::array<metricType, 3> vector = {1.0f, 0.0f, 0.0f};
        
        auto bodyToEarth = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, vector);
        
        auto result = bodyToEarth->result_getter();
        
        metricType cos45 = std::cos(static_cast<metricType>(M_PI / 4.0));
        metricType sin45 = std::sin(static_cast<metricType>(M_PI / 4.0));
        
        REQUIRE(result[0] == Approx(cos45).margin(1e-6f));
        REQUIRE(result[1] == Approx(0.0f).margin(1e-6f));
        REQUIRE(result[2] == Approx(-sin45).margin(1e-6f));
    }
}

TEST_CASE("ConeDirection свойства трансформаций", "[math][transformation][properties]") {
    using metricType = float;
    
    SECTION("Свойство обратной трансформации") {
        metricType theta = static_cast<metricType>(M_PI / 6.0);  // 30 градусов
        metricType psi = static_cast<metricType>(M_PI / 4.0);    // 45 градусов
        metricType gamma = static_cast<metricType>(M_PI / 3.0);  // 60 градусов
        
        std::array<metricType, 3> originalVector = {1.0f, 2.0f, 3.0f};
        
        // Прямая трансформация: body -> earth
        auto bodyToEarth = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, originalVector);
        auto earthVector = bodyToEarth->result_getter();
        
        // Обратная трансформация: earth -> body
        auto earthToBody = TransformationFactory<metricType>::createEarthToBodyTransform(
            theta, psi, gamma, earthVector);
        auto recoveredVector = earthToBody->result_getter();
        
        // Должны получить исходный вектор
        REQUIRE(recoveredVector[0] == Approx(originalVector[0]).margin(1e-4f));
        REQUIRE(recoveredVector[1] == Approx(originalVector[1]).margin(1e-4f));
        REQUIRE(recoveredVector[2] == Approx(originalVector[2]).margin(1e-4f));
    }
    
    SECTION("Сохранение длины вектора") {
        metricType theta = static_cast<metricType>(M_PI / 5.0);
        metricType psi = static_cast<metricType>(M_PI / 7.0);
        metricType gamma = static_cast<metricType>(M_PI / 11.0);
        
        std::array<metricType, 3> vector = {3.0f, 4.0f, 5.0f};
        metricType originalMagnitude = std::sqrt(vector[0]*vector[0] + 
                                               vector[1]*vector[1] + 
                                               vector[2]*vector[2]);
        
        auto bodyToEarth = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, vector);
        auto result = bodyToEarth->result_getter();
        
        metricType transformedMagnitude = std::sqrt(result[0]*result[0] + 
                                                  result[1]*result[1] + 
                                                  result[2]*result[2]);
        
        // Ортогональная трансформация должна сохранять длину вектора
        REQUIRE(transformedMagnitude == Approx(originalMagnitude).margin(1e-5f));
    }
    
    SECTION("Тест ортогональности с единичными векторами") {
        metricType theta = static_cast<metricType>(M_PI / 8.0);
        metricType psi = static_cast<metricType>(M_PI / 6.0);
        metricType gamma = static_cast<metricType>(M_PI / 4.0);
        
        // Тестируем ортогональные единичные векторы
        std::array<metricType, 3> ex = {1.0f, 0.0f, 0.0f};
        std::array<metricType, 3> ey = {0.0f, 1.0f, 0.0f};
        std::array<metricType, 3> ez = {0.0f, 0.0f, 1.0f};
        
        auto transform_ex = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, ex);
        auto transform_ey = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, ey);
        auto transform_ez = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, ez);
        
        auto result_ex = transform_ex->result_getter();
        auto result_ey = transform_ey->result_getter();
        auto result_ez = transform_ez->result_getter();
        
        // Проверяем ортогональность трансформированных векторов
        metricType dot_xy = result_ex[0]*result_ey[0] + result_ex[1]*result_ey[1] + result_ex[2]*result_ey[2];
        metricType dot_xz = result_ex[0]*result_ez[0] + result_ex[1]*result_ez[1] + result_ex[2]*result_ez[2];
        metricType dot_yz = result_ey[0]*result_ez[0] + result_ey[1]*result_ez[1] + result_ey[2]*result_ez[2];
        
        REQUIRE(dot_xy == Approx(0.0f).margin(1e-6f));
        REQUIRE(dot_xz == Approx(0.0f).margin(1e-6f));
        REQUIRE(dot_yz == Approx(0.0f).margin(1e-6f));
        
        // Проверяем, что длины остались единичными
        metricType mag_ex = std::sqrt(result_ex[0]*result_ex[0] + result_ex[1]*result_ex[1] + result_ex[2]*result_ex[2]);
        metricType mag_ey = std::sqrt(result_ey[0]*result_ey[0] + result_ey[1]*result_ey[1] + result_ey[2]*result_ey[2]);
        metricType mag_ez = std::sqrt(result_ez[0]*result_ez[0] + result_ez[1]*result_ez[1] + result_ez[2]*result_ez[2]);
        
        REQUIRE(mag_ex == Approx(1.0f).margin(1e-6f));
        REQUIRE(mag_ey == Approx(1.0f).margin(1e-6f));
        REQUIRE(mag_ez == Approx(1.0f).margin(1e-6f));
    }
}

TEST_CASE("ConeDirection граничные случаи и стабильность", "[math][transformation][stability][edge]") {
    using metricType = float;
    
    SECTION("Приближение малых углов") {
        metricType small_angle = 1e-6f;
        std::array<metricType, 3> vector = {1.0f, 1.0f, 1.0f};
        
        auto transform = TransformationFactory<metricType>::createBodyToEarthTransform(
            small_angle, small_angle, small_angle, vector);
        auto result = transform->result_getter();
        
        // При малых углах результат должен быть близок к исходному вектору
        REQUIRE(result[0] == Approx(vector[0]).margin(1e-5f));
        REQUIRE(result[1] == Approx(vector[1]).margin(1e-5f));
        REQUIRE(result[2] == Approx(vector[2]).margin(1e-5f));
    }
    
    SECTION("Стабильность с большими углами") {
        metricType large_angle = static_cast<metricType>(M_PI - 0.1);
        std::array<metricType, 3> vector = {1.0f, 0.0f, 0.0f};
        
        auto transform = TransformationFactory<metricType>::createBodyToEarthTransform(
            large_angle, 0.0f, 0.0f, vector);
        auto result = transform->result_getter();
        
        // Проверяем, что результат имеет правильную длину
        metricType magnitude = std::sqrt(result[0]*result[0] + result[1]*result[1] + result[2]*result[2]);
        REQUIRE(magnitude == Approx(1.0f).margin(1e-5f));
    }
    
    SECTION("Трансформация нулевого вектора") {
        metricType theta = static_cast<metricType>(M_PI / 4.0);
        metricType psi = static_cast<metricType>(M_PI / 3.0);
        metricType gamma = static_cast<metricType>(M_PI / 6.0);
        
        std::array<metricType, 3> zero_vector = {0.0f, 0.0f, 0.0f};
        
        auto transform = TransformationFactory<metricType>::createBodyToEarthTransform(
            theta, psi, gamma, zero_vector);
        auto result = transform->result_getter();
        
        // Трансформация нулевого вектора должна давать нулевой вектор
        REQUIRE(result[0] == Approx(0.0f).margin(1e-6f));
        REQUIRE(result[1] == Approx(0.0f).margin(1e-6f));
        REQUIRE(result[2] == Approx(0.0f).margin(1e-6f));
    }
}