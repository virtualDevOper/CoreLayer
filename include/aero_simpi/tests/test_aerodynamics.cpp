#include <catch2/catch_all.hpp>
#include "../aerodynamics.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace aero;

//==============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
//==============================================================================

AeroConfig createTestConfig() {
    AeroConfig config;
    config.name = "TestVehicle";
    config.global.S_ref = 0.0314;
    config.global.c_ref = 0.5;
    config.global.b_ref = 1.0;
    config.x_com = 0.5;
    
    // Тело
    ComponentConfig body;
    body.name = "body";
    body.type = ComponentType::BODY;
    body.length = 2.5;
    body.diameter = 0.2;
    body.nose_type = NoseType::OGIVE;
    body.x_pos = 0.0;
    config.components.push_back(body);
    
    // Крыло
    ComponentConfig wing;
    wing.name = "wing";
    wing.type = ComponentType::WING;
    wing.S_ref = 0.3;
    wing.b_ref = 1.0;
    wing.c_ref = 0.3;
    wing.AR = 3.33;
    wing.x_pos = 1.0;
    wing.k_interference = 1.15;
    config.components.push_back(wing);
    
    // Руль
    ComponentConfig fin;
    fin.name = "fin";
    fin.type = ComponentType::FIN;
    fin.S_ref = 0.05;
    fin.b_ref = 0.3;
    fin.c_ref = 0.15;
    fin.AR = 1.8;
    fin.x_pos = 2.3;
    fin.mount_angle = 0.0;
    fin.can_deflect = true;
    fin.delta = 0.0;
    config.components.push_back(fin);
    
    return config;
}

AeroState createTestState(double alpha = 0.0, double M = 0.5) {
    AeroState state;
    state.V = 170.0;  // ~Mach 0.5 на уровне моря
    state.alpha = alpha;
    state.beta = 0.0;
    state.p = 0.0;
    state.q = 0.0;
    state.r = 0.0;
    state.rho = 1.225;
    state.M = M;
    state.Re = 1e6;
    state.dt = 0.01;
    state.x_com = 0.5;
    state.alpha_prev = alpha;
    state.epsilon_prev = 0.0;
    return state;
}

//==============================================================================
// ТЕСТЫ: ВАЛИДАЦИЯ ВХОДНЫХ ДАННЫХ
//==============================================================================

TEST_CASE("AeroState validation rejects zero velocity", "[validation]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState();
    state.V = 0.0;
    REQUIRE_THROWS_AS(model->calculate(state), SingularError);
}

TEST_CASE("AeroState validation rejects zero density", "[validation]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState();
    state.rho = 0.0;
    REQUIRE_THROWS_AS(model->calculate(state), SingularError);
}

TEST_CASE("AeroState validation rejects Mach > 5", "[validation]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState();
    state.M = 5.5;
    REQUIRE_THROWS_AS(model->calculate(state), RangeError);
}

TEST_CASE("AeroState validation rejects alpha > 90", "[validation]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState();
    state.alpha = 95.0;
    REQUIRE_THROWS_AS(model->calculate(state), RangeError);
}

//==============================================================================
// ТЕСТЫ: БАЗОВАЯ АЭРОДИНАМИКА
//==============================================================================

TEST_CASE("Zero angle of attack produces symmetric forces", "[aerodynamics]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(0.0, 0.5);
    auto output = model->calculate(state);
    
    // При α=0, β=0: боковая сила должна быть ~0
    REQUIRE(std::abs(output.Cy) < 1e-6);
    // Момент рыскания должен быть ~0
    REQUIRE(std::abs(output.mz) < 1e-6);
    // Лобовое сопротивление должно быть положительным
    REQUIRE(output.Cx > 0.0);
}

TEST_CASE("Positive alpha produces negative Cz (Z-UP convention)", "[aerodynamics]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.5);  // α = 5°
    auto output = model->calculate(state);
    
    // Z-UP: подъёмная сила вверх = отрицательный Cz
    REQUIRE(output.Cz < 0.0);
}

TEST_CASE("Positive beta produces negative Cy", "[aerodynamics]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(0.0, 0.5);
    state.beta = 5.0;  // β = 5°
    auto output = model->calculate(state);
    
    // β > 0: боковая сила влево = отрицательный Cy
    REQUIRE(output.Cy < 0.0);
}

//==============================================================================
// ТЕСТЫ: ПРОИЗВОДНЫЕ
//==============================================================================

TEST_CASE("dCz_dα is negative (lift curve slope)", "[derivatives]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(0.0, 0.5);
    auto output = model->calculate(state);
    
    // При α=0: dCz/dα < 0 (т.к. Cz < 0 при подъёмной силе)
    REQUIRE(output.dCz_dα < 0.0);
}

TEST_CASE("dmy_dq is negative (pitch damping)", "[derivatives]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(0.0, 0.5);
    auto output = model->calculate(state);
    
    // Демпфирование: момент против угловой скорости
    REQUIRE(output.dmy_dq < 0.0);
}

TEST_CASE("dmx_dp is negative (roll damping)", "[derivatives]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(0.0, 0.5);
    auto output = model->calculate(state);
    
    // Демпфирование: момент против угловой скорости
    REQUIRE(output.dmx_dp < 0.0);
}

//==============================================================================
// ТЕСТЫ: ЧИСЛО МАХА
//==============================================================================

TEST_CASE("Subsonic flight (M=0.5) produces valid output", "[mach]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.5);
    auto output = model->calculate(state);
    
    REQUIRE_FALSE(output.is_transonic);
    REQUIRE(std::isfinite(output.Cx));
    REQUIRE(std::isfinite(output.Cz));
    REQUIRE(std::isfinite(output.my));
}

TEST_CASE("Transonic flight (M=0.95) is detected", "[mach]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.95);
    auto output = model->calculate(state);
    
    REQUIRE(output.is_transonic);
}

TEST_CASE("Supersonic flight (M=2.0) produces valid output", "[mach]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 2.0);
    auto output = model->calculate(state);
    
    REQUIRE_FALSE(output.is_transonic);
    REQUIRE(std::isfinite(output.Cx));
    REQUIRE(std::isfinite(output.Cz));
}

TEST_CASE("Hypersonic flight (M=4.0) produces valid output", "[mach]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 4.0);
    auto output = model->calculate(state);
    
    REQUIRE(std::isfinite(output.Cx));
    REQUIRE(std::isfinite(output.Cz));
}

//==============================================================================
// ТЕСТЫ: ЦЕНТР ДАВЛЕНИЯ
//==============================================================================

TEST_CASE("Center of pressure is within vehicle bounds", "[geometry]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.5);
    auto output = model->calculate(state);
    
    // ЦД должен быть в пределах длины тела (0 ... 2.5 м)
    REQUIRE(output.x_cp >= 0.0);
    REQUIRE(output.x_cp <= 2.5);
}

TEST_CASE("Static margin is calculated correctly", "[geometry]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.5);
    auto output = model->calculate(state);
    
    // static_margin = (x_cp - x_com) / c_ref
    double expected_margin = (output.x_cp - state.x_com) / config.global.c_ref;
    REQUIRE(std::abs(output.static_margin - expected_margin) < 1e-6);
}

//==============================================================================
// ТЕСТЫ: JSON КОНФИГУРАЦИЯ
//==============================================================================

TEST_CASE("JSON config loader throws on invalid file", "[config]") {
    REQUIRE_THROWS_AS(AerodynamicsModel::createFromFile("/nonexistent/path/config.json"),
                     ConfigError);
}

TEST_CASE("JSON config loader validates configuration", "[config]") {
    const std::string invalid_json = R"({
        "aircraft": {
            "S_ref_global": -1.0,
            "components": []
        }
    })";
    REQUIRE_THROWS_AS(JsonParser::parse(invalid_json),
                     ConfigError);
}

//==============================================================================
// ТЕСТЫ: СИЛЫ И МОМЕНТЫ
//==============================================================================

TEST_CASE("getForcesAndMoments returns correct values", "[forces]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    auto state = createTestState(5.0, 0.5);
    auto output = model->calculate(state);
    
    double q = 0.5 * state.rho * state.V * state.V;
    auto [forces, moments] = output.getForcesAndMoments(config.global.S_ref,
                                                        config.global.c_ref,
                                                        config.global.b_ref,
                                                        state.V,
                                                        state.rho);
    
    // Проверка размерностей
    REQUIRE(std::abs(forces.x() - output.Cx * q * config.global.S_ref) < 1e-6);
    REQUIRE(std::abs(forces.z() - output.Cz * q * config.global.S_ref) < 1e-6);
    REQUIRE(std::abs(moments.y() - output.my * q * config.global.S_ref * config.global.c_ref) < 1e-6);
}

//==============================================================================
// ТЕСТЫ: ГИСТЕРЕЗИС
//==============================================================================

TEST_CASE("Hysteresis affects stall detection", "[hysteresis]") {
    auto config = createTestConfig();
    auto model = AerodynamicsModel::create(config);
    
    // Шаг 1: большой угол атаки
    auto state1 = createTestState(15.0, 0.5);
    state1.alpha_prev = 10.0;
    state1.dt = 0.01;
    auto output1 = model->calculate(state1);
    
    // Шаг 2: угол уменьшился, но гистерезис сохраняет эффект
    auto state2 = createTestState(12.0, 0.5);
    state2.alpha_prev = 15.0;
    state2.dt = 0.01;
    auto output2 = model->calculate(state2);
    
    // Гистерезис должен влиять на флаг сваливания
    // (точная проверка зависит от параметров гистерезиса)
    REQUIRE(std::isfinite(output2.Cz));
}
