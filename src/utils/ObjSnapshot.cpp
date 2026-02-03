#include "PCH.h"
#include "ObjSnapshot.h"

template<typename metricType>
ObjSnapshot<metricType>::ObjSnapshot()
    : kinematics_(KinematicState<metricType>::createBuilder().build()),
      time_(std::numeric_limits<metricType>::quiet_NaN()),
      mass_(std::numeric_limits<metricType>::quiet_NaN()),
      inertia_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
      totalForce_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
      totalMoment_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
      totalForceEarth_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())),
      totalMomentEarth_(Eigen::Vector3<metricType>::Constant(std::numeric_limits<metricType>::quiet_NaN())) {}

// Delegate to kinematics for backward compatibility
template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getPosition() const {
    return kinematics_.getPosition();
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getVelocity() const {
    return kinematics_.getVelocity();
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getEulerAngles() const {
    return kinematics_.getEulerAngles();
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getAngularVelocity() const {
    return kinematics_.getAngularVelocity();
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getTime() const {
    return time_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getMass() const {
    return mass_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getInertia() const {
    return inertia_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getTotalForce() const {
    return totalForce_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getTotalMoment() const {
    return totalMoment_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getTotalForceEarth() const {
    return totalForceEarth_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& ObjSnapshot<metricType>::getTotalMomentEarth() const {
    return totalMomentEarth_;
}

template<typename metricType>
const KinematicState<metricType>& ObjSnapshot<metricType>::getKinematics() const {
    return kinematics_;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setTime(metricType t) {
    time_ = t;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setMass(metricType m) {
    mass_ = m;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setInertia(const Eigen::Vector3<metricType>& i) {
    inertia_ = i;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setTotalForce(const Eigen::Vector3<metricType>& f) {
    totalForce_ = f;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setTotalMoment(const Eigen::Vector3<metricType>& m) {
    totalMoment_ = m;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setTotalForceEarth(const Eigen::Vector3<metricType>& f) {
    totalForceEarth_ = f;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setTotalMomentEarth(const Eigen::Vector3<metricType>& m) {
    totalMomentEarth_ = m;
    return *this;
}

template<typename metricType>
std::unique_ptr<ObjSnapshot<metricType>> ObjSnapshot<metricType>::Builder::buildUnique() const {
    auto snapshot = std::make_unique<ObjSnapshot<metricType>>();
    snapshot->kinematics_ = kinematics_;
    snapshot->time_ = time_;
    snapshot->mass_ = mass_;
    snapshot->inertia_ = inertia_;
    snapshot->totalForce_ = totalForce_;
    snapshot->totalMoment_ = totalMoment_;
    snapshot->totalForceEarth_ = totalForceEarth_;
    snapshot->totalMomentEarth_ = totalMomentEarth_;
    return snapshot;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder ObjSnapshot<metricType>::createBuilder(const KinematicState<metricType>& kinematics) {
    return Builder::fromKinematics(kinematics);
}

template<typename metricType>
std::map<std::string, metricType> ObjSnapshot<metricType>::getParams() const {
    std::map<std::string, metricType> params;
    
    if (!std::isnan(time_)) params["time"] = time_;

    const auto& pos = kinematics_.getPosition();
    const auto& vel = kinematics_.getVelocity();
    const auto& euler = kinematics_.getEulerAngles();
    const auto& ang_vel = kinematics_.getAngularVelocity();
    
    params["position_x"] = pos.x(); params["position_y"] = pos.y(); params["position_z"] = pos.z();
    params["velocity_x"] = vel.x(); params["velocity_y"] = vel.y(); params["velocity_z"] = vel.z();
    params["eulerAngles_x"] = euler.x(); params["eulerAngles_y"] = euler.y(); params["eulerAngles_z"] = euler.z();
    params["angularVelocity_x"] = ang_vel.x(); params["angularVelocity_y"] = ang_vel.y(); params["angularVelocity_z"] = ang_vel.z();
    
    if (!std::isnan(mass_)) params["mass"] = mass_;
    if (!std::isnan(inertia_.x())) params["inertia_x"] = inertia_.x();
    if (!std::isnan(inertia_.y())) params["inertia_y"] = inertia_.y();
    if (!std::isnan(inertia_.z())) params["inertia_z"] = inertia_.z();
    if (!std::isnan(totalForce_.x())) params["totalForce_x"] = totalForce_.x();
    if (!std::isnan(totalForce_.y())) params["totalForce_y"] = totalForce_.y();
    if (!std::isnan(totalForce_.z())) params["totalForce_z"] = totalForce_.z();
    if (!std::isnan(totalMoment_.x())) params["totalMoment_x"] = totalMoment_.x();
    if (!std::isnan(totalMoment_.y())) params["totalMoment_y"] = totalMoment_.y();
    if (!std::isnan(totalMoment_.z())) params["totalMoment_z"] = totalMoment_.z();
    
    // Earth frame forces and moments if available
    if (!std::isnan(totalForceEarth_.x())) params["totalForceEarth_x"] = totalForceEarth_.x();
    if (!std::isnan(totalForceEarth_.y())) params["totalForceEarth_y"] = totalForceEarth_.y();
    if (!std::isnan(totalForceEarth_.z())) params["totalForceEarth_z"] = totalForceEarth_.z();
    if (!std::isnan(totalMomentEarth_.x())) params["totalMomentEarth_x"] = totalMomentEarth_.x();
    if (!std::isnan(totalMomentEarth_.y())) params["totalMomentEarth_y"] = totalMomentEarth_.y();
    if (!std::isnan(totalMomentEarth_.z())) params["totalMomentEarth_z"] = totalMomentEarth_.z();
    
    return params;
}

// Explicit template instantiation
template class ObjSnapshot<GLOBAL_CONFIG::PROJECT_TYPE>;