#pragma once

#include "types.h"
#include "config.h"
#include "component.h"
#include "parser.h"
#include <vector>
#include <memory>

namespace aero {

/**
 * @brief Основной класс аэродинамической модели (SOLID: Single Responsibility)
 * 
 * Этот класс orchestrates все компоненты и выполняет итоговое суммирование
 * аэродинамических коэффициентов с учётом интерференции и весовых функций.
 * 
 * @warning НЕ является потокобезопасным из-за кэша гистерезиса.
 *          Для параллельных расчётов создайте отдельный экземпляр на поток.
 */
class AerodynamicsModel {
private:
    AeroConfig config_;
    std::vector<std::shared_ptr<IAerodynamicComponent>> components_;
    
    // Кэшированные значения для гистерезиса
    mutable double alpha_prev_{0.0};
    mutable double epsilon_prev_{0.0};
    
    /**
     * @brief Применение интерференции между компонентами
     */
    void applyInterference(std::vector<AeroOutput>& outputs, const AeroState& state) const;
    
    /**
     * @brief Гибридное суммирование с весовыми функциями
     */
    AeroOutput sumComponents(const std::vector<AeroOutput>& outputs,
                            const AeroState& state) const;
    
    /**
     * @brief Вычисление epsilon (скос потока) из выходных данных компонентов
     * @param outputs Результаты расчёта компонентов
     * @param state Текущее состояние полёта
     * @return Вычисленное значение epsilon для обновления кэша гистерезиса
     */
    double calculateEpsilonFromOutputs(const std::vector<AeroOutput>& outputs,
                                      const AeroState& state) const;
    
    /**
     * @brief Расчёт весовых коэффициентов по числу Маха
     */
    struct MachWeights {
        double sub{1.0};    // Дозвук
        double sup{0.0};    // Сверхзвук
        double hyper{0.0};  // Гиперзвук
    };
    MachWeights calculateMachWeights(double M) const;
    
public:
    /**
     * @brief Конструктор
     * @param config Конфигурация модели
     */
    explicit AerodynamicsModel(const AeroConfig& config);
    
    /**
     * @brief Статический фабричный метод
     * @param config Конфигурация модели
     * @return Умный указатель на модель
     */
    static std::shared_ptr<AerodynamicsModel> create(const AeroConfig& config);
    
    /**
     * @brief Статический фабричный метод из JSON файла
     * @param filepath Путь к JSON файлу конфигурации
     * @return Умный указатель на модель
     * @throws ConfigError если файл не найден или невалиден
     */
    static std::shared_ptr<AerodynamicsModel> createFromFile(const std::string& filepath);
    
    /**
     * @brief Основной метод расчёта аэродинамики
     * @param state Состояние полёта
     * @return Результаты расчёта
     * @throws SingularError если состояние невалидно
     * @throws RangeError если параметры вне диапазона
     * 
     * @warning Изменяет внутреннее состояние гистерезиса (alpha_prev_, epsilon_prev_).
     *          Не вызывайте из нескольких потоков одновременно.
     * @note После вызова обновляет внутренний кэш гистерезиса для следующего шага.
     *       Не вызывайте несколько раз на одном шаге симуляции с одинаковым state.
     */
    AeroOutput calculate(const AeroState& state) const;
    
    /**
     * @brief Получить конфигурацию модели
     */
    const AeroConfig& getConfig() const {
        return config_;
    }
    
    /**
     * @brief Получить количество компонентов
     */
    size_t getComponentCount() const {
        return components_.size();
    }
    
    /**
     * @brief Сброс состояния гистерезиса
     */
    void resetState() const {
        alpha_prev_ = 0.0;
        epsilon_prev_ = 0.0;
    }
};

} // namespace aero
