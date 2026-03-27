#include "RigidBody.hpp"

#include "sim/engine/Integrator.hpp"

namespace sim {
namespace dynamics {

RigidBody::RigidBody(const common::Matrix3& inertia, const common::Quaternion& init_att,
                     const common::Vector3& init_omega)
    : inertia_(inertia), inertia_inv_(inertia.inverse()) {
    state_.resize(7);
    // Eigen quaternion coeffs: (x, y, z, w)
    state_.segment<4>(0) = init_att.coeffs();
    state_.segment<3>(4) = init_omega;
}

common::VectorX RigidBody::dynamics(double, const common::VectorX& y, const common::Vector3& external_torque,
                                    const common::Vector3& internal_torque, const common::Vector3& internal_momentum) {
    // Unpack state
    common::Vector4 q_coeffs = y.segment<4>(0);
    common::Quaternion q(q_coeffs(3), q_coeffs(0), q_coeffs(1), q_coeffs(2));  // w, x, y, z construction
    q.normalize();                                                             // Ensure unit quaternion

    common::Vector3 omega = y.segment<3>(4);

    // Attitude Kinematics: q_dot = 0.5 * q * omega_quat
    // Using Eigen multiplication:
    common::Quaternion omega_q(0, omega.x(), omega.y(), omega.z());
    common::Quaternion q_dot = q * omega_q;
    q_dot.coeffs() *= 0.5;

    // Attitude Dynamics: I * omega_dot + omega x (I * omega + H_int) = tau_ext + tau_int
    // omega_dot = I_inv * (tau_ext + tau_int - omega x (I * omega + H_int))
    common::Vector3 total_h = inertia_ * omega + internal_momentum;
    common::Vector3 w_cross_h = omega.cross(total_h);
    common::Vector3 omega_dot = inertia_inv_ * (external_torque + internal_torque - w_cross_h);

    common::VectorX dydt(7);
    dydt.segment<4>(0) = q_dot.coeffs();
    dydt.segment<3>(4) = omega_dot;

    return dydt;
}

void RigidBody::step(double dt, const common::Vector3& external_torque, const common::Vector3& internal_torque,
                     const common::Vector3& internal_momentum) {
    auto f = [this, external_torque, internal_torque, internal_momentum](double t, const common::VectorX& y) {
        return this->dynamics(t, y, external_torque, internal_torque, internal_momentum);
    };

    state_ = engine::Integrator<common::VectorX>::rk4(0.0, state_, dt, f);

    // Normalize quaternion after integration step to prevent drift
    state_.segment<4>(0).normalize();
}

common::Quaternion RigidBody::getAttitude() const {
    common::Vector4 c = state_.segment<4>(0);
    return common::Quaternion(c(3), c(0), c(1), c(2));  // w, x, y, z
}

common::Vector3 RigidBody::getAngularVelocity() const {
    return state_.segment<3>(4);
}

}  // namespace dynamics
}  // namespace sim
