#include <cassert>
#include "../src/utils/ObjManager/ObjectManager.h"
#include "../src/PhysicalObjects/AbstractObject.h"
#include "../src/DynamicsSystem/IDynamicsSystem.h"
#include "../src/utils/ObjInitParams.h"
#include "../src/utils/KinematicState.h"

// Простейшая фиктивная система ОДУ без физики
template<typename T>
class DummySystem final : public IDynamicsSystem<T> {
public:
    ~DummySystem() override = default;

    std::string get_description() override {
        return "dummy";
    }

    std::unique_ptr<KinematicState<T>> get_rhs_derivatives(
        const KinematicState<T>& state,
        T /*t*/
    ) override {
        // Возвращаем нулевые производные, чтобы не вмешиваться в физику
        auto zero = KinematicState<T>::createBuilder()
            .setPosition(Eigen::Vector3<T>::Zero())
            .setVelocity(Eigen::Vector3<T>::Zero())
            .setEulerAngles(Eigen::Vector3<T>::Zero())
            .setAngularVelocity(Eigen::Vector3<T>::Zero())
            .build();
        return std::make_unique<KinematicState<T>>(zero);
    }

    std::unique_ptr<ObjSnapshot<T>> augmentSnapshot(
        const KinematicState<T>& kinematics,
        T /*t*/
    ) const override {
        return ObjSnapshot<T>::createBuilder(kinematics).buildUnique();
    }
};

// Фиктивный объект, чтобы протестировать менеджер
template<typename T>
class DummyObject final : public AbstractObject<T> {
public:
    DummyObject()
        : AbstractObject<T>(
              std::make_unique<DummySystem<T>>(),
              std::make_unique<ObjInitParams<T>>()) {}
};

int main() {
    using M = float;
    ObjectManager<M> manager;

    auto obj = std::make_shared<DummyObject<M>>();
    int id = manager.addTrackedObject(obj);
    assert(manager.getObjectCount() == 1);

    auto fetched = manager.getObjectByID(id);
    assert(fetched != nullptr);
    assert(fetched.get() == obj.get());

    bool removed = manager.removeObjectById(id);
    assert(removed);
    assert(manager.getObjectCount() == 0);

    manager.clearAllObjects();
    assert(manager.getObjectCount() == 0);

    return 0;
}

