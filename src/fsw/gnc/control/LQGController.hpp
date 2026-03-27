#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>

#include "fsw/gnc/control/LQRController.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief Linear Quadratic Gaussian (LQG) controller.
 *
 * Combines Multiplicative Extended Kalman Filter (MEKF) for state estimation
 * and Linear Quadratic Regulator (LQR) for optimal control.
 */
class LQGController {
   public:
    struct Config {
        LQRController::Config lqr_cfg;

        Config() : lqr_cfg(LQRController::Config::Default()) {}
    };

    /**
     * @brief Construct a new LQGController object
     * @param mekf Pointer to the MEKF instance providing state estimates
     * @param config LQG configuration
     */
    LQGController(std::shared_ptr<ekf::MEKF> mekf, const Config& config) : mekf_(mekf), lqr_(config.lqr_cfg) {}

    /**
     * @brief Compute optimal control torque
     *
     * @param q_target Target attitude (Inertial->Body)
     * @param omega_target Target angular velocity in Body frame [rad/s]
     * @param omega_meas Raw gyro measurement [rad/s]
     * @return common::Vector3 Commanded torque in Body frame [Nm]
     */
    common::Vector3 computeTorque(const common::Quaternion& q_target, const common::Vector3& omega_target,
                                  const common::Vector3& omega_meas) {
        if (!mekf_) return common::Vector3::Zero();

        // 1. Get current estimate from MEKF
        common::Quaternion q_est = mekf_->getAttitude();  // I->B representation
        common::Vector3 beta_est = mekf_->getBias();

        // 2. Compute attitude error
        // Both q_est and q_target are Inertial->Body quaternions
        common::Quaternion q_err = q_est * q_target.inverse();

        // Ensure q_err is shortest path
        if (q_err.w() < 0) {
            q_err.coeffs() *= -1.0;
        }

        // Use full axis-angle representation for better accuracy with large rotations
        common::Vector3 delta_theta;
        double error_angle = 2.0 * std::acos(std::min(1.0, std::abs(q_err.w())));

        if (error_angle < 1e-6) {
            // Near zero error, use small-angle approximation
            delta_theta = 2.0 * q_err.vec();
        } else {
            // Full quaternion-to-axis-angle conversion
            double sin_half = std::sin(error_angle / 2.0);
            if (std::abs(sin_half) > 1e-9) {
                delta_theta = (error_angle / sin_half) * q_err.vec();
            } else {
                delta_theta = 2.0 * q_err.vec();
            }
        }

        // 3. Compute angular velocity estimate and error
        common::Vector3 omega_est = omega_meas - beta_est;
        common::Vector3 delta_omega = omega_est - omega_target;

        // 4. Form state vector x for LQR
        common::VectorX x(6);
        x << delta_theta, delta_omega;

        // 5. Compute LQR torque
        common::Vector3 torque = lqr_.computeTorque(x);

        return torque;
    }

    /**
     * @brief Update LQR gains
     */
    void setGains(const common::MatrixX& K) {
        lqr_.setGains(K);
    }

    /**
     * @brief Access the underlying LQR controller
     */
    LQRController& getLQR() {
        return lqr_;
    }

   private:
    std::shared_ptr<ekf::MEKF> mekf_;
    LQRController lqr_;
};

}  // namespace control
}  // namespace gnc
}  // namespace fsw
