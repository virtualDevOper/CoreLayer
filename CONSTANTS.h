//
// Created by 4NR_Operator_3 on 16.09.2025.
//

#pragma once
#include "GLOBAL_CONFIG.h"


namespace PhysicsConstants {
    constexpr GLOBAL_CONFIG::PROJECT_TYPE rho0 = 1.225;          // Плотность на уровне моря [kg/m³]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE T0 = 288.15;           // Температура на уровне моря [K]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE L = 0.0065;            // Градиент температуры [K/m]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE g = 9.80665;           // Ускорение свободного падения [m/s²]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE molarMass = 0.0289644; // Молярная масса воздуха [kg/mol] (переименовал M для избежания конфликта)
    constexpr GLOBAL_CONFIG::PROJECT_TYPE R = 8.31447;           // Универсальная газовая постоянная [J/(mol·K)]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE P0 = 101325.0;         // Давление на уровне моря [Pa]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE gamma = 1.4;           // Коэффициент Пуассона
    constexpr GLOBAL_CONFIG::PROJECT_TYPE maxHeight = 11000.0;  // Максимальная высота (начинается тропосфера) [m]
    // здесь предполагается, что задача нацелена на маленькие ракеты(МБР тут не рассматриваем,
    // там совсем другая динамика полета). То, как работать с большими ракетами - секретный секрет(((
    constexpr GLOBAL_CONFIG::PROJECT_TYPE minHeight = 0.0;      // Минимальная высота [m]

    constexpr GLOBAL_CONFIG::PROJECT_TYPE earthRadius = 6.371e6;     // Радиус Земли [м]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE gravitationalConstant = 6.67430e-11; // Гравитационная постоянная [м³/кг·с²]
    constexpr GLOBAL_CONFIG::PROJECT_TYPE earthMass = 5.972e24;      // Масса Земли [кг]
}
