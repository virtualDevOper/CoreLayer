//
// Created by 4NR_Operator_3 on 03.10.2025.
//

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
    ObjSnapshot(const ObjSnapshot&) = delete;
    ObjSnapshot& operator=(const ObjSnapshot&) = delete;
    ObjSnapshot(ObjSnapshot&&) = default;
    ObjSnapshot& operator=(ObjSnapshot&&) = default;
    ~ObjSnapshot() = default;

    const Eigen::Vector3<metricType>& getEulerAngles() const { return eulerAngles_; }
    const Eigen::Vector3<metricType>& getPosition() const { return position_; }
    const Eigen::Vector3<metricType>& getVelocity() const { return velocity_; }
    const Eigen::Vector3<metricType>& getAngularVelocity() const { return angularVelocity_; }
    const Eigen::Vector3<metricType>& getInertia() const { return inertia_; }
    const Eigen::Vector3<metricType>& getTotalForce() const { return totalForce_; }
    const Eigen::Vector3<metricType>& getTotalMoment() const { return totalMoment_; }
    metricType getMass() const { return mass_; }


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

        /**
         * \brief Создает ObjSnapshot с текущими настройками
         * \return Уникальный указатель на созданный снапшот
         */
        std::unique_ptr<ObjSnapshot<metricType>> build() {
            return std::unique_ptr<ObjSnapshot<metricType>>(
                new ObjSnapshot<metricType>(
                    mass_, inertia_, totalForce_, totalMoment_,
                    position_, velocity_, eulerAngles_,
                    angularVelocity_
                )
            );
        }

        /**
         * \brief Создает ObjSnapshot с текущими настройками (альтернатива с shared_ptr)
         */
        std::shared_ptr<ObjSnapshot<metricType>> buildShared() {
            return std::make_shared<ObjSnapshot<metricType>>(
                mass_, inertia_, totalForce_, totalMoment_,
                position_, velocity_, eulerAngles_,
                angularVelocity_
            );
        }

    private:
        metricType mass_ = 0.0;
        Eigen::Vector3<metricType> inertia_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> totalForce_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> totalMoment_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> position_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> velocity_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> eulerAngles_ = Eigen::Matrix<metricType, 3, 1>::Zero();
        Eigen::Vector3<metricType> angularVelocity_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    };

    /**
     * \brief Создает строитель для ObjSnapshot
     * \return Builder для настройки параметров
     */
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
        const Eigen::Vector3<metricType>& angularVelocity
    ) : mass_(mass), inertia_(inertia),
        totalForce_(totalForce), totalMoment_(totalMoment), position_(position),
        velocity_(velocity),  eulerAngles_(eulerAngles),
        angularVelocity_(angularVelocity) {}

    metricType mass_ = 0.0;
    Eigen::Vector3<metricType> inertia_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> totalForce_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> totalMoment_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> position_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> velocity_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> eulerAngles_ = Eigen::Matrix<metricType, 3, 1>::Zero();
    Eigen::Vector3<metricType> angularVelocity_ = Eigen::Matrix<metricType, 3, 1>::Zero();
};


/*
// Создание снапшота с использованием Builder
auto snapshot = ObjSnapshot<double>::createBuilder()
    .setPosition(Eigen::Vector3d(1.0, 2.0, 3.0))
    .setVelocity(Eigen::Vector3d(10.0, 0.0, 0.0))
    .setEulerAngles(Eigen::Vector3d(0.1, 0.2, 0.3))
    .setMass(100.0)
    .setAcceleration(Eigen::Vector3d(0.0, 0.0, -9.8))
    .build();

// Или с shared_ptr
auto sharedSnapshot = ObjSnapshot<double>::createBuilder()
    .setPosition(Eigen::Vector3d(1.0, 2.0, 3.0))
    .setVelocity(Eigen::Vector3d(10.0, 0.0, 0.0))
    .buildShared();

// Использование снапшота
std::cout << "Position: " << snapshot->getPosition().transpose() << std::endl;
std::cout << "Mass: " << snapshot->getMass() << std::endl;*/