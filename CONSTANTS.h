//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once

namespace PhysicsConstants {
    constexpr double rho0 = 1.225;          // Плотность на уровне моря [kg/m³]
    constexpr double T0 = 288.15;           // Температура на уровне моря [K]
    constexpr double L = 0.0065;            // Градиент температуры [K/m]
    constexpr double g = 9.80665;           // Ускорение свободного падения [m/s²]
    constexpr double molarMass = 0.0289644; // Молярная масса воздуха [kg/mol] (переименовал M для избежания конфликта)
    constexpr double R = 8.31447;           // Универсальная газовая постоянная [J/(mol·K)]
    constexpr double P0 = 101325.0;         // Давление на уровне моря [Pa]
    constexpr double gamma = 1.4;           // Коэффициент Пуассона
    constexpr double maxHeight = 11000.0;  // Максимальная высота (начинается тропосфера) [m]
    // здесь предполагается, что задача нацелена на маленькие ракеты(МБР тут не рассматриваем,
    // там совсем другая динамика полета). То, как работать с большими ракетами - секретный секрет(((
    constexpr double minHeight = 0.0;      // Минимальная высота [m]

    constexpr double earthRadius = 6.371e6;     // Радиус Земли [м]
    constexpr double gravitationalConstant = 6.67430e-11; // Гравитационная постоянная [м³/кг·с²]
    constexpr double earthMass = 5.972e24;      // Масса Земли [кг]
}
