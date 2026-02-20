#pragma once

#include "types.h"
#include "config.h"
#include <memory>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace aero {

/**
 * @brief Интерфейс аэродинамического компонента (SOLID: Interface Segregation)
 */
class IAerodynamicComponent {
public:
    virtual ~IAerodynamicComponent() = default;
    
    /**
     * @brief Вычисление аэродинамических коэффициентов компонента
     * @param state Состояние полёта
     * @param global Глобальная конфигурация
     * @return AeroOutput с коэффициентами компонента
     */
    virtual AeroOutput calculate(const AeroState& state, const GlobalConfig& global) const = 0;
    
    /**
     * @brief Получить тип компонента
     */
    virtual ComponentType getType() const = 0;
    
    /**
     * @brief Получить имя компонента
     */
    virtual std::string getName() const = 0;
};

/**
 * @brief Базовый класс для всех компонентов (SOLID: Single Responsibility)
 */
class AerodynamicComponent : public IAerodynamicComponent {
protected:
    ComponentConfig config_;
    
public:
    explicit AerodynamicComponent(const ComponentConfig& config) : config_(config) {}
    virtual ~AerodynamicComponent() = default;
    
    std::string getName() const override {
        return config_.name;
    }
    
    ComponentType getType() const override {
        return config_.type;
    }
    
protected:
    /**
     * @brief Конверсия градусов в радианы
     */
    static double deg2rad(double deg) {
        return deg * M_PI / 180.0;
    }
    
    /**
     * @brief Конверсия радиан в градусы
     */
    static double rad2deg(double rad) {
        return rad * 180.0 / M_PI;
    }
    
    /**
     * @brief Знак числа
     */
    static int sign(double val) {
        return (val > 0) - (val < 0);
    }
    
    /**
     * @brief Сигмоидальная функция для плавной интерполяции
     */
    static double sigmoid(double x, double center = 0.0, double width = 1.0) {
        return 1.0 / (1.0 + std::exp(-(x - center) / width));
    }
};

/**
 * @brief Компонент: Тело вращения (SOLID: Single Responsibility)
 */
class BodyComponent : public AerodynamicComponent {
public:
    explicit BodyComponent(const ComponentConfig& config);
    AeroOutput calculate(const AeroState& state, const GlobalConfig& global) const override;
};

/**
 * @brief Компонент: Крыло (SOLID: Single Responsibility)
 */
class WingComponent : public AerodynamicComponent {
public:
    explicit WingComponent(const ComponentConfig& config);
    AeroOutput calculate(const AeroState& state, const GlobalConfig& global) const override;
};

/**
 * @brief Компонент: Руль/Стабилизатор (SOLID: Single Responsibility)
 */
class FinComponent : public AerodynamicComponent {
public:
    explicit FinComponent(const ComponentConfig& config);
    AeroOutput calculate(const AeroState& state, const GlobalConfig& global) const override;
};

/**
 * @brief Фабрика компонентов (SOLID: Dependency Inversion)
 */
class ComponentFactory {
public:
    static std::shared_ptr<IAerodynamicComponent> create(const ComponentConfig& config);
};

} // namespace aero
