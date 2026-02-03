#include <catch2/catch_test_macros.hpp>

/**
 * Пример мок теста
 * Тестирует компоненты с использованием mock объектов
 */

// Простой mock класс для демонстрации
class MockComponent {
public:
    bool called = false;
    
    void mockMethod() {
        called = true;
    }
    
    bool wasCalled() const {
        return called;
    }
};

TEST_CASE("Mock Test Example", "[mock]") {
    SECTION("Mock object behavior") {
        MockComponent mock;
        
        REQUIRE_FALSE(mock.wasCalled());
        
        mock.mockMethod();
        
        REQUIRE(mock.wasCalled());
    }
    
    SECTION("Mock interaction test") {
        // Тест взаимодействия с mock объектами
        MockComponent mock;
        
        // Симуляция вызова
        mock.mockMethod();
        
        REQUIRE(mock.wasCalled());
    }
}

TEST_CASE("Advanced Mock Testing", "[mock][advanced]") {
    SECTION("Multiple mock objects") {
        MockComponent mock1, mock2;
        
        mock1.mockMethod();
        
        REQUIRE(mock1.wasCalled());
        REQUIRE_FALSE(mock2.wasCalled());
    }
}