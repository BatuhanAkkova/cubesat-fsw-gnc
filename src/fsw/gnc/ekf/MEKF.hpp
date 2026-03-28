#pragma once

#include "fsw/gnc/interfaces/IEstimator.hpp"

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
class MEKF : public interfaces::IEstimator {
   public:
    MEKF();

    /**
     * @brief Update the filter with new sensor data.
     */
    void update(const common::SensorData& sensors, double dt) override;

    /**
     * @brief Initialize the filter
     */
    void initialize(const common::Quaternion& q0, const common::Vector3& beta0, const common::MatrixX& P0);

    /**
     * @brief Prediction Step (Time Update)
     */
    void predict(const common::Vector3& gyro_meas, double dt, const common::MatrixX& Q);

    /**
     * @brief Update Step with Quaternion Measurement (Star Tracker)
     */
    void update_quat(const common::Quaternion& q_meas, const common::Matrix3& R);

    /**
     * @brief Reset filter to initial conditions
     */
    void reset() override;

    /**
     * @brief Get latest estimates
     */
    common::Quaternion getAttitude() const override {
        return q_est_;
    }
    common::Vector3 getAngularVelocity() const override {
        return w_est_;
    }
    bool isValid() const override {
        return !P_.array().isNaN().any();
    }

    common::Vector3 getBias() const {
        return beta_est_;
    }
    common::MatrixX getCovariance() const {
        return P_;
    }

   private:
    common::Quaternion q_est_;  // Nominal attitude (Inertial -> Body)
    common::Vector3 beta_est_;  // Nominal gyro bias
    common::Vector3 w_est_;     // Net angular velocity estimate
    common::MatrixX P_;         // Error covariance (6x6)
};

}  // namespace ekf
}  // namespace gnc
}  // namespace fsw
