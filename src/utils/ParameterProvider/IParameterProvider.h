//
// Created by refactoring on 29.01.2026.
//
#pragma once
#include "PCH.h"

/**
* \brief Интерфейс для провайдера динамических параметров объекта.
*
* \tparam metricType Тип данных для метрических величин.
*
* \details Абстракция для получения изменяющихся во времени параметров
* физических объектов: массы, моментов инерции и тяги. Позволяет
* разорвать циклические зависимости между системами динамики и
* объектами, применяя принцип инверсии зависимостей (DIP).
* Поддерживает интерполяцию параметров по времени.
*/
template<typename metricType>
class IMassProvider {
public:
    [[nodiscard]] virtual std::optional<metricType> tryGetMass(metricType t) const = 0;
    [[nodiscard]] virtual bool hasMass() const = 0;
    virtual ~IMassProvider() = default;
};

template<typename metricType>
class IInertiaProvider {
public:
    [[nodiscard]] virtual std::optional<Eigen::Vector3<metricType>> tryGetInertia(metricType t) const = 0;
    [[nodiscard]] virtual bool hasInertia() const = 0;
    virtual ~IInertiaProvider() = default;
};

template<typename metricType>
class IThrustProvider {
public:
    [[nodiscard]] virtual std::optional<Eigen::Vector3<metricType>> tryGetThrust(metricType t) const = 0;
    [[nodiscard]] virtual bool hasThrust() const = 0;
    virtual ~IThrustProvider() = default;
};

template<typename metricType>
class ICOMProvider {
public:
    [[nodiscard]] virtual std::optional<Eigen::Vector3<metricType>> tryGetCOM(metricType t) const = 0;
    [[nodiscard]] virtual bool hasCOM() const = 0;
    virtual ~ICOMProvider() = default;
};

// === ИСПРАВЛЕНО: убрано virtual, т.к. нет ромбовидного наследования ===
// ComponentInterpolationManager наследуется ТОЛЬКО от IParameterProvider,
// не от базовых интерфейсов напрямую → ромба нет → virtual избыточен
template<typename metricType>
class IParameterProvider
: public IMassProvider<metricType>
, public IInertiaProvider<metricType>
, public IThrustProvider<metricType>
, public ICOMProvider<metricType>
{
public:
    // === Старый контракт (бросает исключения) — для обратной совместимости ===
    [[nodiscard]] virtual metricType getMass(metricType t) const = 0;
    [[nodiscard]] virtual Eigen::Vector3<metricType> getInertia(metricType t) const = 0;
    [[nodiscard]] virtual Eigen::Vector3<metricType> getThrust(metricType t) const = 0;
    [[nodiscard]] virtual Eigen::Vector3<metricType> getCOM(metricType t) const = 0;

    // === Реализация новых интерфейсов через старый контракт ===
    [[nodiscard]] std::optional<metricType> tryGetMass(metricType t) const override {
        try { return getMass(t); }
        catch (...) { return std::nullopt; }
    }
    [[nodiscard]] bool hasMass() const override {
        try { getMass(metricType{0}); return true; }
        catch (...) { return false; }
    }
    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetInertia(metricType t) const override {
        try { return getInertia(t); }
        catch (...) { return std::nullopt; }
    }
    [[nodiscard]] bool hasInertia() const override {
        try { getInertia(metricType{0}); return true; }
        catch (...) { return false; }
    }
    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetThrust(metricType t) const override {
        try { return getThrust(t); }
        catch (...) { return std::nullopt; }
    }
    [[nodiscard]] bool hasThrust() const override {
        try { getThrust(metricType{0}); return true; }
        catch (...) { return false; }
    }
    [[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetCOM(metricType t) const override {
        try { return getCOM(t); }
        catch (...) { return std::nullopt; }
    }
    [[nodiscard]] bool hasCOM() const override {
        try { getCOM(metricType{0}); return true; }
        catch (...) { return false; }
    }
    virtual ~IParameterProvider() = default;
};