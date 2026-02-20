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
      aero_Cx_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_Cy_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_Cz_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_mx_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_my_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_mz_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_alpha_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_beta_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_mach_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_x_cp_(std::numeric_limits<metricType>::quiet_NaN()),
      aero_static_margin_(std::numeric_limits<metricType>::quiet_NaN())
{}

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

// === ГЕТТЕРЫ АЭРОДИНАМИЧЕСКИХ КОЭФФИЦИЕНТОВ ===
template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroCx() const {
    return aero_Cx_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroCy() const {
    return aero_Cy_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroCz() const {
    return aero_Cz_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroMx() const {
    return aero_mx_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroMy() const {
    return aero_my_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroMz() const {
    return aero_mz_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroAlpha() const {
    return aero_alpha_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroBeta() const {
    return aero_beta_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroMach() const {
    return aero_mach_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroXcp() const {
    return aero_x_cp_;
}

template<typename metricType>
metricType ObjSnapshot<metricType>::getAeroStaticMargin() const {
    return aero_static_margin_;
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

// === СЕТТЕРЫ АЭРОДИНАМИЧЕСКИХ КОЭФФИЦИЕНТОВ ===
template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroCx(metricType cx) {
    aero_Cx_ = cx;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroCy(metricType cy) {
    aero_Cy_ = cy;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroCz(metricType cz) {
    aero_Cz_ = cz;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroMx(metricType mx) {
    aero_mx_ = mx;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroMy(metricType my) {
    aero_my_ = my;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroMz(metricType mz) {
    aero_mz_ = mz;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroAlpha(metricType alpha) {
    aero_alpha_ = alpha;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroBeta(metricType beta) {
    aero_beta_ = beta;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroMach(metricType mach) {
    aero_mach_ = mach;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroXcp(metricType x_cp) {
    aero_x_cp_ = x_cp;
    return *this;
}

template<typename metricType>
typename ObjSnapshot<metricType>::Builder& ObjSnapshot<metricType>::Builder::setAeroStaticMargin(metricType static_margin) {
    aero_static_margin_ = static_margin;
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
    
    // Аэродинамические коэффициенты
    snapshot->aero_Cx_ = aero_Cx_;
    snapshot->aero_Cy_ = aero_Cy_;
    snapshot->aero_Cz_ = aero_Cz_;
    snapshot->aero_mx_ = aero_mx_;
    snapshot->aero_my_ = aero_my_;
    snapshot->aero_mz_ = aero_mz_;
    snapshot->aero_alpha_ = aero_alpha_;
    snapshot->aero_beta_ = aero_beta_;
    snapshot->aero_mach_ = aero_mach_;
    snapshot->aero_x_cp_ = aero_x_cp_;
    snapshot->aero_static_margin_ = aero_static_margin_;
    
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
    
    // Аэродинамические коэффициенты
    if (!std::isnan(aero_Cx_)) params["aero_Cx"] = aero_Cx_;
    if (!std::isnan(aero_Cy_)) params["aero_Cy"] = aero_Cy_;
    if (!std::isnan(aero_Cz_)) params["aero_Cz"] = aero_Cz_;
    if (!std::isnan(aero_mx_)) params["aero_mx"] = aero_mx_;
    if (!std::isnan(aero_my_)) params["aero_my"] = aero_my_;
    if (!std::isnan(aero_mz_)) params["aero_mz"] = aero_mz_;
    if (!std::isnan(aero_alpha_)) params["aero_alpha"] = aero_alpha_;
    if (!std::isnan(aero_beta_)) params["aero_beta"] = aero_beta_;
    if (!std::isnan(aero_mach_)) params["aero_mach"] = aero_mach_;
    if (!std::isnan(aero_x_cp_)) params["aero_x_cp"] = aero_x_cp_;
    if (!std::isnan(aero_static_margin_)) params["aero_static_margin"] = aero_static_margin_;

    return params;
}

// Explicit template instantiation
template class ObjSnapshot<GLOBAL_CONFIG::PROJECT_TYPE>;