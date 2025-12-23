#pragma once
#include "../../PCH.h"

/**
 * \brief Снимок состояния объекта
 * 
 * \tparam metricType Тип данных для метрических величин
 * 
 * \detail Класс является immutable.
 * Для создания используется Builder.
 * У меня возникла дилема с этим классом, по идее такие параметры, как аэродинамический угол
 * крена или пространственный угол атаки нужен только для определения АД коэффициентов.
 * Так-то он вычисляется через скорости, поэтому хранить его тут не вижу смысла,
 * если где-то потребуются эти величины, то их спокойно можно вычислить.
 */

template <typename metricType>
class ObjSnapshot {
public:
    // РАЗРЕШАЕМ копирование для работы с StateStorage
    ObjSnapshot(const ObjSnapshot&) = default;
    ObjSnapshot& operator=(const ObjSnapshot&) = default;
    ObjSnapshot(ObjSnapshot&&) noexcept = default;
    ObjSnapshot& operator=(ObjSnapshot&&) noexcept = default;
    ~ObjSnapshot() = default;

    // Геттеры
    const Eigen::Vector3<metricType>& getEulerAngles() const { return eulerAngles_; }
    const Eigen::Vector3<metricType>& getPosition() const { return position_; }
    const Eigen::Vector3<metricType>& getVelocity() const { return velocity_; }
    const Eigen::Vector3<metricType>& getAngularVelocity() const { return angularVelocity_; }
    const Eigen::Vector3<metricType>& getInertia() const { return inertia_; }
    const Eigen::Vector3<metricType>& getTotalForce() const { return totalForce_; }
    const Eigen::Vector3<metricType>& getTotalMoment() const { return totalMoment_; }
    metricType getMass() const { return mass_; }

    // Оператор сложения
    ObjSnapshot operator+(const ObjSnapshot& other) const {
        return Builder()
            .setPosition(position_ + other.position_)
            .setVelocity(velocity_ + other.velocity_)
            .setEulerAngles(eulerAngles_ + other.eulerAngles_)
            .setAngularVelocity(angularVelocity_ + other.angularVelocity_)
            .setMass(mass_)  // Масса не суммируется
            .setInertia(inertia_)  // Инерция не суммируется
            .setTotalForce(totalForce_ + other.totalForce_)
            .setTotalMoment(totalMoment_ + other.totalMoment_)
            .build();
    }

    // Оператор умножения на скаляр (справа)
    ObjSnapshot operator*(metricType scalar) const {
        return Builder()
            .setPosition(position_ * scalar)
            .setVelocity(velocity_ * scalar)
            .setEulerAngles(eulerAngles_ * scalar)
            .setAngularVelocity(angularVelocity_ * scalar)
            .setMass(mass_)  // Масса не умножается
            .setInertia(inertia_)  // Инерция не умножается
            .setTotalForce(totalForce_ * scalar)
            .setTotalMoment(totalMoment_ * scalar)
            .build();
    }

    // Оператор умножения на скаляр (слева) - friend функция
    friend ObjSnapshot operator*(metricType scalar, const ObjSnapshot& snapshot) {
        return snapshot * scalar;  // Используем уже определенный оператор
    }

    // Метод getParams для CsvSaveStrategy
    std::map<std::string, metricType> getParams() const {
        std::map<std::string, metricType> params;

        // Позиция
        params["position_x"] = position_.x();
        params["position_y"] = position_.y();
        params["position_z"] = position_.z();

        // Скорость
        params["velocity_x"] = velocity_.x();
        params["velocity_y"] = velocity_.y();
        params["velocity_z"] = velocity_.z();

        // Углы Эйлера
        params["eulerAngles_x"] = eulerAngles_.x();
        params["eulerAngles_y"] = eulerAngles_.y();
        params["eulerAngles_z"] = eulerAngles_.z();

        // Угловая скорость
        params["angularVelocity_x"] = angularVelocity_.x();
        params["angularVelocity_y"] = angularVelocity_.y();
        params["angularVelocity_z"] = angularVelocity_.z();

        // Моменты инерции
        params["inertia_x"] = inertia_.x();
        params["inertia_y"] = inertia_.y();
        params["inertia_z"] = inertia_.z();

        // Силы
        params["totalForce_x"] = totalForce_.x();
        params["totalForce_y"] = totalForce_.y();
        params["totalForce_z"] = totalForce_.z();

        // Моменты
        params["totalMoment_x"] = totalMoment_.x();
        params["totalMoment_y"] = totalMoment_.y();
        params["totalMoment_z"] = totalMoment_.z();

        // Масса
        params["mass"] = mass_;

        return params;
    }

    class Builder {
    public:
        Builder() = default;

        Builder& setEulerAngles(const Eigen::Vector3<metricType>& eulerAngles) {
            eulerAngles_ = eulerAngles;
            return *this;
        }

        Builder& setPosition(const Eigen::Vector3<metricType>& position) {
            position_ = position;
            return *this;
        }

        Builder& setTotalForce(const Eigen::Vector3<metricType>& totalForce) {
            totalForce_ = totalForce;
            return *this;
        }

        Builder& setTotalMoment(const Eigen::Vector3<metricType>& totalMoment) {
            totalMoment_ = totalMoment;
            return *this;
        }

        Builder& setVelocity(const Eigen::Vector3<metricType>& velocity) {
            velocity_ = velocity;
            return *this;
        }

        Builder& setAngularVelocity(const Eigen::Vector3<metricType>& angularVelocity) {
            angularVelocity_ = angularVelocity;
            return *this;
        }

        Builder& setInertia(const Eigen::Vector3<metricType>& inertia) {
            inertia_ = inertia;
            return *this;
        }

        Builder& setMass(metricType mass) {
            mass_ = mass;
            return *this;
        }

        // Создает уникальный указатель
        std::unique_ptr<ObjSnapshot> buildUnique() {
            return std::unique_ptr<ObjSnapshot>(
                new ObjSnapshot(
                    mass_, inertia_, totalForce_, totalMoment_,
                    position_, velocity_, eulerAngles_,
                    angularVelocity_, std::move(setParameters_)
                )
            );
        }

        // Создает shared указатель
        std::shared_ptr<ObjSnapshot> buildShared() {
            return std::make_shared<ObjSnapshot>(
                mass_, inertia_, totalForce_, totalMoment_,
                position_, velocity_, eulerAngles_,
                angularVelocity_, std::move(setParameters_)
            );
        }

        // Создает по значению (для операторов)
        ObjSnapshot build() {
            return ObjSnapshot(
                mass_, inertia_, totalForce_, totalMoment_,
                position_, velocity_, eulerAngles_,
                angularVelocity_, std::move(setParameters_)
            );
        }

    private:


        metricType mass_ = 0.0;
        Eigen::Vector3<metricType> inertia_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> totalForce_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> totalMoment_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> position_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> velocity_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> eulerAngles_ = Eigen::Vector3<metricType>::Zero();
        Eigen::Vector3<metricType> angularVelocity_ = Eigen::Vector3<metricType>::Zero();
        std::vector<std::string> setParameters_;
    };

    // Статический метод для создания Builder
    static Builder createBuilder() {
        return Builder();
    }

private:
    ObjSnapshot(
        metricType mass,
        const Eigen::Vector3<metricType>& inertia,
        const Eigen::Vector3<metricType>& totalForce,
        const Eigen::Vector3<metricType>& totalMoment,
        const Eigen::Vector3<metricType>& position,
        const Eigen::Vector3<metricType>& velocity,
        const Eigen::Vector3<metricType>& eulerAngles,
        const Eigen::Vector3<metricType>& angularVelocity,
        std::vector<std::string> setParameters
    ) : mass_(mass), inertia_(inertia),
        totalForce_(totalForce), totalMoment_(totalMoment),
        position_(position), velocity_(velocity),
        eulerAngles_(eulerAngles), angularVelocity_(angularVelocity),
        setParameters_(std::move(setParameters)) {}

    metricType mass_ = 0.0;
    Eigen::Vector3<metricType> inertia_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> totalForce_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> totalMoment_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> position_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> velocity_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> eulerAngles_ = Eigen::Vector3<metricType>::Zero();
    Eigen::Vector3<metricType> angularVelocity_ = Eigen::Vector3<metricType>::Zero();
    std::vector<std::string> setParameters_;
};