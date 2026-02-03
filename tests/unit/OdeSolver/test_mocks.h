#pragma once

#include <memory>
#include <map>
#include <functional>
#include "../../../src/DynamicsSystem/IDynamicsSystem.h"
#include "../../../src/utils/ObjManager/IObjectManager.h"
#include "../../../src/SimulationMomento/SimulationMomento.h"
#include "../../../src/PhysicalObjects/AbstractObject.h"
#include "../../../src/utils/ObjInitParams.h"

/**
 * \brief Мок системы динамики для аналитических тестовых случаев
 * 
 * \details Позволяет задать произвольную функцию производных через лямбду.
 * Используется для тестирования решателя на известных аналитических решениях.
 */
template<typename metricType>
class MockDynamicsSystem : public IDynamicsSystem<metricType> {
private:
    std::function<KinematicState<metricType>(const KinematicState<metricType>&, metricType)> derivative_func_;
    std::string description_;
    
public:
    /**
     * \brief Конструктор мок системы динамики
     * \param func Функция, вычисляющая производные состояния
     * \param desc Описание системы
     */
    explicit MockDynamicsSystem(
        std::function<KinematicState<metricType>(const KinematicState<metricType>&, metricType)> func,
        const std::string& desc = "Мок системы динамики")
        : derivative_func_(std::move(func)), description_(desc) {}
    
    std::string get_description() const override {
        return description_;
    }
    
    std::unique_ptr<KinematicState<metricType>> get_rhs_derivatives(
        const KinematicState<metricType>& state,
        metricType t
    ) override {
        auto result = derivative_func_(state, t);
        return std::make_unique<KinematicState<metricType>>(std::move(result));
    }
    
    std::unique_ptr<ObjSnapshot<metricType>> augmentSnapshot(
        const KinematicState<metricType>& kinematics,
        metricType t
    ) const override {
        return ObjSnapshot<metricType>::createBuilder(kinematics)
            .setTime(t)
            .setMass(metricType(1.0))
            .setInertia(Eigen::Vector3<metricType>(metricType(1.0), metricType(1.0), metricType(1.0)))
            .setTotalForce(Eigen::Vector3<metricType>::Zero())
            .setTotalMoment(Eigen::Vector3<metricType>::Zero())
            .setTotalForceEarth(Eigen::Vector3<metricType>::Zero())
            .setTotalMomentEarth(Eigen::Vector3<metricType>::Zero())
            .buildUnique();
    }
};

/**
 * \brief Мок физического объекта для тестирования
 * 
 * \details Наследуется от AbstractObject и правильно инициализирует базовый класс.
 * Позволяет управлять состоянием активности объекта для тестирования.
 */
template<typename metricType>
class MockObject : public AbstractObject<metricType> {
private:
    std::shared_ptr<IDynamicsSystem<metricType>> dynamics_shared_;  // Храним shared_ptr для копирования
    
public:
    /**
     * \brief Конструктор мок объекта
     * \param dynamics Система динамики (будет скопирована в unique_ptr)
     * \param init_pos Начальная позиция объекта
     * \param active Начальное состояние активности
     */
    explicit MockObject(
        std::shared_ptr<IDynamicsSystem<metricType>> dynamics,
        const Eigen::Vector3<metricType>& init_pos = Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0)),
        bool active = true)
        : AbstractObject<metricType>(
            createDynamicsPtr(dynamics),
            createInitParams(init_pos)
        ),
        dynamics_shared_(dynamics)
    {
        if (active) {
            this->setActive();
        } else {
            this->setDestroyed();
        }
    }
    
    /**
     * \brief Изменить состояние активности объекта
     */
    void setActiveState(bool active) {
        if (active) {
            this->setActive();
        } else {
            this->setDestroyed();
        }
    }

private:
    /**
     * \brief Создает unique_ptr для системы динамики
     * \details Создает новый экземпляр MockDynamicsSystem с той же функцией производных
     */
    static std::unique_ptr<IDynamicsSystem<metricType>> createDynamicsPtr(
        std::shared_ptr<IDynamicsSystem<metricType>> dynamics)
    {
        // Получаем указатель на MockDynamicsSystem
        auto* mock_dyn = dynamic_cast<MockDynamicsSystem<metricType>*>(dynamics.get());
        if (!mock_dyn) {
            throw std::runtime_error("Dynamics system must be MockDynamicsSystem");
        }
        
        // Создаем копию MockDynamicsSystem
        return std::make_unique<MockDynamicsSystem<metricType>>(*mock_dyn);
    }
    
    /**
     * \brief Создает начальные параметры объекта
     */
    static std::unique_ptr<ObjInitParams<metricType>> createInitParams(
        const Eigen::Vector3<metricType>& init_pos)
    {
        auto params = std::make_unique<ObjInitParams<metricType>>();
        params->position = init_pos;
        params->velocity = Eigen::Vector3<metricType>::Zero();
        params->eulerAngles = Eigen::Vector3<metricType>::Zero();
        params->angularVelocity = Eigen::Vector3<metricType>::Zero();
        return params;
    }
};

/**
 * \brief Мок менеджера объектов для тестирования
 * 
 * \details Реализует интерфейс IObjectManager для управления тестовыми объектами.
 * Хранит объекты через weak_ptr для соответствия реальному интерфейсу.
 */
template<typename metricType>
class MockObjectManager : public IObjectManager<metricType> {
private:
    std::map<int, std::shared_ptr<AbstractObject<metricType>>> objects_storage_;  // Для владения
    std::map<int, std::weak_ptr<AbstractObject<metricType>>> objects_;            // Для интерфейса
    
public:
    /**
     * \brief Добавить объект с автоматическим ID
     */
    int addTrackedObject(std::shared_ptr<AbstractObject<metricType>> object) override {
        int id = static_cast<int>(objects_storage_.size()) + 1;
        objects_storage_[id] = object;
        objects_[id] = object;
        return id;
    }
    
    /**
     * \brief Добавить объект с заданным ID (для тестов)
     */
    void addObject(int id, std::shared_ptr<AbstractObject<metricType>> obj) {
        objects_storage_[id] = obj;
        objects_[id] = obj;
    }
    
    /**
     * \brief Получить объект по ID
     */
    std::shared_ptr<AbstractObject<metricType>> getObjectByID(int id) const override {
        auto it = objects_.find(id);
        if (it != objects_.end()) {
            return it->second.lock();
        }
        return nullptr;
    }
    
    /**
     * \brief Удалить объект по ID
     */
    bool removeObjectById(int id) override {
        objects_storage_.erase(id);
        return objects_.erase(id) > 0;
    }
    
    /**
     * \brief Получить количество объектов
     */
    size_t getObjectCount() const override {
        return objects_.size();
    }
    
    /**
     * \brief Очистить все объекты
     */
    void clearAllObjects() override {
        objects_storage_.clear();
        objects_.clear();
    }
    
    /**
     * \brief Получить все объекты (для RungeKutta4Solver)
     */
    std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>> getAllObjects() const override {
        std::vector<std::pair<int, std::weak_ptr<AbstractObject<metricType>>>> result;
        for (const auto& [id, obj] : objects_) {
            result.emplace_back(id, obj);
        }
        return result;
    }
};

/**
 * \brief Вспомогательная функция для создания мок системы динамики
 * 
 * \details Упрощает создание систем динамики для типичных тестовых случаев.
 */
template<typename metricType>
std::shared_ptr<MockDynamicsSystem<metricType>> createExponentialDecaySystem(metricType lambda = metricType(1.0)) {
    return std::make_shared<MockDynamicsSystem<metricType>>(
        [lambda](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
            // Производная: dx/dt = -lambda * x (экспоненциальное затухание)
            return KinematicState<metricType>::createBuilder()
                .setPosition(-lambda * state.getPosition())
                .setVelocity(Eigen::Vector3<metricType>::Zero())
                .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                .build();
        },
        "Система экспоненциального затухания"
    );
}

/**
 * \brief Вспомогательная функция для создания системы экспоненциального роста
 */
template<typename metricType>
std::shared_ptr<MockDynamicsSystem<metricType>> createExponentialGrowthSystem(metricType lambda = metricType(0.5)) {
    return std::make_shared<MockDynamicsSystem<metricType>>(
        [lambda](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
            // Производная: dx/dt = lambda * x (экспоненциальный рост)
            return KinematicState<metricType>::createBuilder()
                .setPosition(lambda * state.getPosition())
                .setVelocity(Eigen::Vector3<metricType>::Zero())
                .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                .build();
        },
        "Система экспоненциального роста"
    );
}

/**
 * \brief Вспомогательная функция для создания линейной системы
 */
template<typename metricType>
std::shared_ptr<MockDynamicsSystem<metricType>> createLinearSystem() {
    return std::make_shared<MockDynamicsSystem<metricType>>(
        [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
            // Производная: dx/dt = t (интеграл: x = t²/2 + C)
            return KinematicState<metricType>::createBuilder()
                .setPosition(Eigen::Vector3<metricType>(t, metricType(0.0), metricType(0.0)))
                .setVelocity(Eigen::Vector3<metricType>::Zero())
                .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                .build();
        },
        "Линейная система динамики"
    );
}

/**
 * \brief Вспомогательная функция для создания константной системы
 */
template<typename metricType>
std::shared_ptr<MockDynamicsSystem<metricType>> createConstantSystem() {
    return std::make_shared<MockDynamicsSystem<metricType>>(
        [](const KinematicState<metricType>& state, metricType t) -> KinematicState<metricType> {
            // Производная: dx/dt = 1 (интеграл: x = t + C)
            return KinematicState<metricType>::createBuilder()
                .setPosition(Eigen::Vector3<metricType>(metricType(1.0), metricType(0.0), metricType(0.0)))
                .setVelocity(Eigen::Vector3<metricType>::Zero())
                .setEulerAngles(Eigen::Vector3<metricType>::Zero())
                .setAngularVelocity(Eigen::Vector3<metricType>::Zero())
                .build();
        },
        "Константная система динамики"
    );
}
