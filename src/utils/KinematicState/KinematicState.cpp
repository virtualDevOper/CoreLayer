#include "PCH.h"
#include "KinematicState.h"

template<typename metricType>
typename KinematicState<metricType>::Builder& KinematicState<metricType>::Builder::setPosition(const Eigen::Vector3<metricType>& v) {
    position_ = v;
    return *this;
}

template<typename metricType>
typename KinematicState<metricType>::Builder& KinematicState<metricType>::Builder::setVelocity(const Eigen::Vector3<metricType>& v) {
    velocity_ = v;
    return *this;
}

template<typename metricType>
typename KinematicState<metricType>::Builder& KinematicState<metricType>::Builder::setEulerAngles(const Eigen::Vector3<metricType>& v) {
    eulerAngles_ = v;
    return *this;
}

template<typename metricType>
typename KinematicState<metricType>::Builder& KinematicState<metricType>::Builder::setAngularVelocity(const Eigen::Vector3<metricType>& v) {
    angularVelocity_ = v;
    return *this;
}

template<typename metricType>
KinematicState<metricType> KinematicState<metricType>::Builder::build() const {
    return KinematicState(position_, velocity_, eulerAngles_, angularVelocity_);
}

template<typename metricType>
typename KinematicState<metricType>::Builder KinematicState<metricType>::createBuilder() {
    return Builder();
}

template<typename metricType>
const Eigen::Vector3<metricType>& KinematicState<metricType>::getPosition() const {
    return position_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& KinematicState<metricType>::getVelocity() const {
    return velocity_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& KinematicState<metricType>::getEulerAngles() const {
    return eulerAngles_;
}

template<typename metricType>
const Eigen::Vector3<metricType>& KinematicState<metricType>::getAngularVelocity() const {
    return angularVelocity_;
}

template<typename metricType>
KinematicState<metricType> KinematicState<metricType>::operator+(const KinematicState& other) const {
    return KinematicState(
        position_ + other.position_,
        velocity_ + other.velocity_,
        eulerAngles_ + other.eulerAngles_,
        angularVelocity_ + other.angularVelocity_
    );
}

template<typename metricType>
KinematicState<metricType> KinematicState<metricType>::operator*(metricType scalar) const {
    return KinematicState(
        position_ * scalar,
        velocity_ * scalar,
        eulerAngles_ * scalar,
        angularVelocity_ * scalar
    );
}

template<typename metricType>
KinematicState<metricType>::KinematicState(
    Eigen::Vector3<metricType> position,
    Eigen::Vector3<metricType> velocity,
    Eigen::Vector3<metricType> eulerAngles,
    Eigen::Vector3<metricType> angularVelocity
) : position_(std::move(position)),
    velocity_(std::move(velocity)),
    eulerAngles_(std::move(eulerAngles)),
    angularVelocity_(std::move(angularVelocity)) {}

// Global operator for scalar * state symmetry
template<typename metricType>
KinematicState<metricType> operator*(metricType scalar, const KinematicState<metricType>& state) {
    return state * scalar;
}

// Explicit template instantiation
template class KinematicState<GLOBAL_CONFIG::PROJECT_TYPE>;
template KinematicState<GLOBAL_CONFIG::PROJECT_TYPE> operator*(GLOBAL_CONFIG::PROJECT_TYPE, const KinematicState<GLOBAL_CONFIG::PROJECT_TYPE>&);