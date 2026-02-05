#include "MEKF.hpp"
#include "common/logger.hpp"
#include <iostream>

namespace fsw {
namespace gnc {
namespace ekf {

MEKF::MEKF() {
    q_est_.setIdentity();
    beta_est_.setZero();
    P_.setIdentity(6, 6);
}

void MEKF::initialize(const common::Quaternion& q0, const common::Vector3& beta0, const common::MatrixX& P0) {
    q_est_ = q0;
    q_est_.normalize();
    beta_est_ = beta0;
    P_ = P0;
}

void MEKF::predict(const common::Vector3& gyro_meas, double dt, const common::MatrixX& Q) {
    // 1. Propagate Nominal State
    common::Vector3 omega_est = gyro_meas - beta_est_;
    
    // Kinematics for q_IB (Inertial -> Body): q_dot = -0.5 * omega * q
    common::Quaternion omega_q(0, omega_est.x(), omega_est.y(), omega_est.z()); // w, x, y, z
    
    // Eigen q1 * q2 is quaternion multiplication
    // q_dot = -0.5 * omega_q * q_est_
    common::Quaternion q_dot = omega_q * q_est_;
    q_dot.coeffs() *= -0.5;

    // Simple Euler integration for now (or midpoint)
    q_est_.coeffs() += q_dot.coeffs() * dt;
    q_est_.normalize();

    // Beta constant in prediction
    
    // 2. Propagate Covariance
    // F matrix (continuous time linearized dynamics)
    // d(delta_theta)/dt = -[omega x] * delta_theta - delta_beta
    // d(delta_beta)/dt = 0
    
    common::MatrixX F(6, 6);
    F.setZero();
    
    // Top-left: -[omega x]
    common::Matrix3 omega_cross;
    omega_cross << 0, -omega_est.z(), omega_est.y(),
                   omega_est.z(), 0, -omega_est.x(),
                   -omega_est.y(), omega_est.x(), 0;
                   
    F.block<3, 3>(0, 0) = -omega_cross;
    F.block<3, 3>(0, 3) = common::Matrix3::Identity(); // +I relationship to bias

    // Discretize F -> Phi = I + F*dt
    common::MatrixX Phi = common::MatrixX::Identity(6, 6) + F * dt;

    // P = Phi * P * Phi' + Q
    P_ = Phi * P_ * Phi.transpose() + Q;
}

void MEKF::update_quat(const common::Quaternion& q_meas, const common::Matrix3& R) {
    // 1. Compute Residual
    // dq = q_meas * q_est_inv
    // Ideally dq should be close to identity
    common::Quaternion dq = q_meas * q_est_.inverse();
    
    // Residual vector z = 2 * dq.vec (vector part)
    // Small angle approximation: dq = [1, 0.5*theta]
    common::Vector3 z = 2.0 * dq.vec();
    
    // Ensure we take the short path (if w < 0, negate to keep close to +Identity)
    // Note: dq.vec() sign depends on w. If dq.w() < 0, it represents same rotation but far path. 
    // Usually q and -q are same. To treat as small error, we want w ~= 1.
    if (dq.w() < 0) {
        z = -z;
    }

    // 2. Sensitivity Matrix H
    // z = H * x + v
    // H = [I  0]
    common::MatrixX H(3, 6);
    H.setZero();
    H.block<3, 3>(0, 0) = common::Matrix3::Identity();

    // 3. Kalman Gain
    // S = H * P * H' + R
    common::MatrixX S = H * P_ * H.transpose() + R;
    common::MatrixX K = P_ * H.transpose() * S.inverse();

    // 4. Update State
    common::VectorX dx = K * z; // 6x1 correction

    common::Vector3 d_theta = dx.segment<3>(0);
    common::Vector3 d_beta = dx.segment<3>(3);

    // Apply corrections
    // Bias: simple addition
    beta_est_ += d_beta;

    // Attitude: q_new = dq(d_theta) * q_old
    // Multiplicative update to preserve unit norm structure better than additive
    // dq = [1, 0.5*d_theta] normalized
    common::Quaternion correction(1.0, d_theta.x() * 0.5, d_theta.y() * 0.5, d_theta.z() * 0.5);
    correction.normalize();
    
    q_est_ = correction * q_est_;
    q_est_.normalize();

    // 5. Update Covariance
    // P = (I - K*H) * P
    common::MatrixX I6 = common::MatrixX::Identity(6, 6);
    P_ = (I6 - K * H) * P_; // Note: Joseph form is more robust but this is standard
}

void MEKF::reset() {
    // Reset to identity attitude and zero bias
    q_est_.setIdentity();
    beta_est_.setZero();
    
    // Set high initial uncertainty (conservative reset)
    P_.setIdentity(6, 6);
    P_ *= 1.0;  // 1 rad^2 for attitude error, 1 rad^2/s^2 for bias
    
    common::LogWarning("[MEKF] Filter reset to initial conditions");
}

} // namespace ekf
} // namespace gnc
} // namespace fsw
