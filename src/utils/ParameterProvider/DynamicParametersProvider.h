//
// Created by refactoring on 12.01.2026.
//
#pragma once
#include "IParameterProvider.h"
#include "PCH.h"
#include "../Interpolation/ComponentInterpolationManager.h"

/**
* \brief Провайдер динамических параметров для системы ОДУ ракеты.
*
* \tparam metricType Тип данных для метрических величин.
*
* \details Агрегирует слабые ссылки на провайдеры массы, инерции, тяги и COM.
* Предоставляет безопасные методы получения параметров с fallback на значения
* по умолчанию. Используется в FullRocketODE для получения изменяющихся
* во времени параметров ракеты без циклических зависимостей.
*/
template<typename metricType>
class DynamicParametersProvider {
private:
std::weak_ptr<IMassProvider<metricType>> mass_provider_;
std::weak_ptr<IInertiaProvider<metricType>> inertia_provider_;
std::weak_ptr<IThrustProvider<metricType>> thrust_provider_;
std::weak_ptr<ICOMProvider<metricType>> com_provider_;

// Внутренний хелпер для безопасного вызова через weak_ptr
// ИСПРАВЛЕНО: корректно обрабатывает std::optional из возвращаемого значения
template<typename ProviderType, typename ReturnType, typename... Args>
[[nodiscard]] static ReturnType safeCall(
    const std::weak_ptr<ProviderType>& weak_ptr,
    ReturnType (ProviderType::*method)(Args...) const,
    Args... args) noexcept
{
    if (auto p = weak_ptr.lock()) {
        try {
            return (p.get()->*method)(args...);
        } catch (...) {
            // Возвращаем пустой optional при исключении
            return ReturnType{};
        }
    }
    // Возвращаем пустой optional при expired weak_ptr
    return ReturnType{};
}

public:
// ========================================================================
// КОНСТРУКТОР
// ========================================================================
DynamicParametersProvider(
    std::shared_ptr<IMassProvider<metricType>> mass = nullptr,
    std::shared_ptr<IInertiaProvider<metricType>> inertia = nullptr,
    std::shared_ptr<IThrustProvider<metricType>> thrust = nullptr,
    std::shared_ptr<ICOMProvider<metricType>> com = nullptr)
: mass_provider_(std::move(mass))
, inertia_provider_(std::move(inertia))
, thrust_provider_(std::move(thrust))
, com_provider_(std::move(com)) {}

// ========================================================================
// БЕЗОПАСНЫЕ ГЕТТЕРЫ С std::optional
// ========================================================================
[[nodiscard]] std::optional<metricType> tryGetMass(metricType t) const noexcept {
    return safeCall(mass_provider_, &IMassProvider<metricType>::tryGetMass, t);
}

[[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetInertia(metricType t) const noexcept {
    return safeCall(inertia_provider_, &IInertiaProvider<metricType>::tryGetInertia, t);
}

[[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetThrust(metricType t) const noexcept {
    return safeCall(thrust_provider_, &IThrustProvider<metricType>::tryGetThrust, t);
}

[[nodiscard]] std::optional<Eigen::Vector3<metricType>> tryGetCOM(metricType t) const noexcept {
    return safeCall(com_provider_, &ICOMProvider<metricType>::tryGetCOM, t);
}

// ========================================================================
// ГЕТТЕРЫ С FALLBACK (удобно для ODE-систем)
// ========================================================================
[[nodiscard]] metricType getMass(metricType t, metricType default_value) const noexcept {
    auto val = tryGetMass(t);
    return val.has_value() ? val.value() : default_value;
}

[[nodiscard]] Eigen::Vector3<metricType> getInertia(
    metricType t, Eigen::Vector3<metricType> default_value) const noexcept {
    auto val = tryGetInertia(t);
    return val.has_value() ? val.value() : default_value;
}

[[nodiscard]] Eigen::Vector3<metricType> getThrust(
    metricType t, Eigen::Vector3<metricType> default_value) const noexcept {
    auto val = tryGetThrust(t);
    return val.has_value() ? val.value() : default_value;
}

[[nodiscard]] Eigen::Vector3<metricType> getCOM(
    metricType t, Eigen::Vector3<metricType> default_value) const noexcept {
    auto val = tryGetCOM(t);
    return val.has_value() ? val.value() : default_value;
}

// ========================================================================
// СТРОГИЕ ГЕТТЕРЫ (бросают, если параметр критичен и отсутствует)
// ========================================================================
[[nodiscard]] metricType getMassStrict(metricType t) const {
    auto val = tryGetMass(t);
    if (!val) {
        throw std::logic_error("Mass provider required but not available");
    }
    return *val;
}

[[nodiscard]] Eigen::Vector3<metricType> getInertiaStrict(metricType t) const {
    auto val = tryGetInertia(t);
    if (!val) {
        throw std::logic_error("Inertia provider required but not available");
    }
    return *val;
}

[[nodiscard]] Eigen::Vector3<metricType> getThrustStrict(metricType t) const {
    auto val = tryGetThrust(t);
    if (!val) {
        throw std::logic_error("Thrust provider required but not available");
    }
    return *val;
}

[[nodiscard]] Eigen::Vector3<metricType> getCOMStrict(metricType t) const {
    auto val = tryGetCOM(t);
    if (!val) {
        throw std::logic_error("COM provider required but not available");
    }
    return *val;
}

// ========================================================================
// ПРОВЕРКИ НАЛИЧИЯ ПРОВАЙДЕРОВ
// ========================================================================
[[nodiscard]] bool hasMassProvider() const noexcept {
    return !mass_provider_.expired();
}

[[nodiscard]] bool hasInertiaProvider() const noexcept {
    return !inertia_provider_.expired();
}

[[nodiscard]] bool hasThrustProvider() const noexcept {
    return !thrust_provider_.expired();
}

[[nodiscard]] bool hasCOMProvider() const noexcept {
    return !com_provider_.expired();
}
};