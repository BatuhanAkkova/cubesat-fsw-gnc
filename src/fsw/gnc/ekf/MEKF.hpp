#pragma once

#include "common/types.hpp"

namespace fsw {
namespace gnc {
namespace ekf {

/**
 * @brief Multiplicative Extended Kalman Filter for Attitude Estimation
 * 
 * State vector (6x1):
 * [0-2]: Attitude Error (delta_theta) [rad]
 * [3-5]: Gyro Bias Error (delta_beta) [rad/s]
 */
class MEKF {
public:
    MEKF();

    /**
     * @brief Initialize the filter
     * 
     * @param q0 Initial attitude estimate (Inertial -> Body)
     * @param beta0 Initial gyro bias estimate [rad/s]
     * @param P0 Initial covariance matrix (6x6)
     */
    void initialize(const common::Quaternion& q0, const common::Vector3& beta0, const common::MatrixX& P0);

    /**
     * @brief Prediction Step (Time Update)
     * 
     * @param gyro_meas Measured angular velocity [rad/s]
     * @param dt Time step [s]
     * @param Q Process noise covariance (6x6)
     */
    void predict(const common::Vector3& gyro_meas, double dt, const common::MatrixX& Q);

    /**
     * @brief Update Step with Quaternion Measurement (Star Tracker)
     * 
     * @param q_meas Measured attitude (Inertial -> Body)
     * @param R Measurement noise covariance (3x3)
     */
    void update_quat(const common::Quaternion& q_meas, const common::Matrix3& R);

    // Getters
    common::Quaternion getAttitude() const { return q_est_; }
    common::Vector3 getBias() const { return beta_est_; }
    common::MatrixX getCovariance() const { return P_; }

private:
    common::Quaternion q_est_; // Nominal attitude (Inertial -> Body)
    common::Vector3 beta_est_; // Nominal gyro bias

    common::MatrixX P_;        // Error covariance (6x6)
};

} // namespace ekf
} // namespace gnc
} // namespace fsw
