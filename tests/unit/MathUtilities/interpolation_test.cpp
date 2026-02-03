 #include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_adapters.hpp>
#include <catch2/generators/catch_generators_random.hpp>

#include "../../../src/utils/Interpolation/ILinearInterpolator/LinearInterpolator/LinearInterpolator.h"
#include <cmath>

using namespace Catch;

TEST_CASE("LinearInterpolator базовая функциональность", "[math][interpolation][linear]") {
    using metricType = float;
    
    SECTION("Простая линейная интерполяция") {
        std::vector<metricType> x_data = {0.0f, 1.0f, 2.0f, 3.0f};
        std::vector<metricType> y_data = {0.0f, 2.0f, 4.0f, 6.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Точки на узлах
        REQUIRE(interp.interpolate(0.0f) == Approx(0.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(1.0f) == Approx(2.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(2.0f) == Approx(4.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(3.0f) == Approx(6.0f).margin(1e-6f));
        
        // Промежуточные точки
        REQUIRE(interp.interpolate(0.5f) == Approx(1.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(1.5f) == Approx(3.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(2.5f) == Approx(5.0f).margin(1e-6f));
    }
    
    SECTION("Интерполяция на неравномерной сетке") {
        std::vector<metricType> x_data = {0.0f, 0.5f, 2.0f, 5.0f};
        std::vector<metricType> y_data = {1.0f, 2.0f, 3.0f, 4.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Проверяем узлы
        REQUIRE(interp.interpolate(0.0f) == Approx(1.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(0.5f) == Approx(2.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(2.0f) == Approx(3.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(5.0f) == Approx(4.0f).margin(1e-6f));
        
        // Промежуточная точка в первом интервале [0, 0.5]
        REQUIRE(interp.interpolate(0.25f) == Approx(1.5f).margin(1e-6f));
        
        // Промежуточная точка в третьем интервале [2, 5]
        REQUIRE(interp.interpolate(3.5f) == Approx(3.5f).margin(1e-6f));
    }
    
    SECTION("Точность интерполяции квадратичной функции") {
        // Тестируем на квадратичной функции y = x^2
        std::vector<metricType> x_data = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
        std::vector<metricType> y_data;
        for (auto x : x_data) {
            y_data.push_back(x * x);
        }
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Линейная интерполяция квадратичной функции не будет точной,
        // но должна давать разумные приближения
        metricType x_test = 1.5f;
        metricType expected = x_test * x_test;  // 2.25
        metricType interpolated = interp.interpolate(x_test);
        
        // Линейная интерполяция между (1,1) и (2,4) в точке 1.5 даст 2.5
        REQUIRE(interpolated == Approx(2.5f).margin(1e-6f));
        
        // Ошибка интерполяции для квадратичной функции
        metricType error = std::abs(interpolated - expected);
        REQUIRE(error == Approx(0.25f).margin(1e-6f));  // |2.5 - 2.25| = 0.25
    }
}

TEST_CASE("LinearInterpolator поведение экстраполяции", "[math][interpolation][extrapolation]") {
    using metricType = float;
    
    std::vector<metricType> x_data = {1.0f, 2.0f, 3.0f};
    std::vector<metricType> y_data = {10.0f, 20.0f, 30.0f};
    
    LinearInterpolator<metricType> interp(x_data, y_data);
    
    SECTION("Левая экстраполяция (константная)") {
        // За пределами слева должно возвращать первое значение
        REQUIRE(interp.interpolate(0.0f) == Approx(10.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(-5.0f) == Approx(10.0f).margin(1e-6f));
    }
    
    SECTION("Правая экстраполяция (константная)") {
        // За пределами справа должно возвращать последнее значение
        REQUIRE(interp.interpolate(4.0f) == Approx(30.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(100.0f) == Approx(30.0f).margin(1e-6f));
    }
}

TEST_CASE("LinearInterpolator граничные случаи и обработка ошибок", "[math][interpolation][edge][error]") {
    using metricType = float;
    
    SECTION("Несовпадающие размеры массивов вызывают исключение") {
        std::vector<metricType> x_data = {1.0f, 2.0f, 3.0f};
        std::vector<metricType> y_data = {10.0f, 20.0f};  // Размер не совпадает
        
        REQUIRE_THROWS_AS(LinearInterpolator<metricType>(x_data, y_data), std::invalid_argument);
    }
    
    SECTION("Пустые массивы вызывают исключение") {
        std::vector<metricType> empty_x;
        std::vector<metricType> empty_y;
        
        LinearInterpolator<metricType> interp(empty_x, empty_y);
        REQUIRE_THROWS_AS(interp.interpolate(1.0f), std::runtime_error);
    }
    
    SECTION("Интерполяция с одной точкой") {
        std::vector<metricType> x_data = {5.0f};
        std::vector<metricType> y_data = {25.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Любая точка должна возвращать единственное значение
        REQUIRE(interp.interpolate(0.0f) == Approx(25.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(5.0f) == Approx(25.0f).margin(1e-6f));
        REQUIRE(interp.interpolate(10.0f) == Approx(25.0f).margin(1e-6f));
    }
    
    SECTION("Обработка дублированных x значений") {
        std::vector<metricType> x_data = {1.0f, 2.0f, 2.0f, 3.0f};  // Дублированное значение
        std::vector<metricType> y_data = {10.0f, 20.0f, 25.0f, 30.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Интерполятор должен работать, но поведение может быть неопределенным
        // Проверяем, что он не падает
        REQUIRE_NOTHROW(interp.interpolate(2.0f));
        REQUIRE_NOTHROW(interp.interpolate(1.5f));
        REQUIRE_NOTHROW(interp.interpolate(2.5f));
    }
}

TEST_CASE("LinearInterpolator математические свойства", "[math][interpolation][properties]") {
    using metricType = float;
    
    SECTION("Сохранение монотонности") {
        // Монотонно возрастающие данные
        std::vector<metricType> x_data = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
        std::vector<metricType> y_data = {1.0f, 3.0f, 5.0f, 7.0f, 9.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Проверяем монотонность на промежуточных точках
        metricType prev_value = interp.interpolate(0.0f);
        for (metricType x = 0.1f; x <= 4.0f; x += 0.1f) {
            metricType current_value = interp.interpolate(x);
            REQUIRE(current_value >= prev_value);  // Должно быть неубывающим
            prev_value = current_value;
        }
    }
    
    SECTION("Линейность в каждом сегменте") {
        std::vector<metricType> x_data = {0.0f, 2.0f, 4.0f};
        std::vector<metricType> y_data = {0.0f, 10.0f, 20.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // В каждом сегменте функция должна быть линейной
        // Проверяем первый сегмент [0, 2]
        metricType y_0_5 = interp.interpolate(0.5f);
        metricType y_1_0 = interp.interpolate(1.0f);
        metricType y_1_5 = interp.interpolate(1.5f);
        
        // Проверяем равномерность приращений
        metricType delta1 = y_1_0 - y_0_5;
        metricType delta2 = y_1_5 - y_1_0;
        
        REQUIRE(delta1 == Approx(delta2).margin(1e-6f));
    }
    
    SECTION("Непрерывность в узлах") {
        std::vector<metricType> x_data = {0.0f, 1.0f, 2.0f, 3.0f};
        std::vector<metricType> y_data = {0.0f, 5.0f, 3.0f, 8.0f};
        
        LinearInterpolator<metricType> interp(x_data, y_data);
        
        // Проверяем непрерывность в узлах
        metricType epsilon = 1e-6f;
        
        for (size_t i = 1; i < x_data.size() - 1; ++i) {
            metricType x_node = x_data[i];
            metricType left_limit = interp.interpolate(x_node - epsilon);
            metricType right_limit = interp.interpolate(x_node + epsilon);
            metricType node_value = interp.interpolate(x_node);
            
            REQUIRE(left_limit == Approx(node_value).margin(1e-4f));
            REQUIRE(right_limit == Approx(node_value).margin(1e-4f));
        }
    }
}