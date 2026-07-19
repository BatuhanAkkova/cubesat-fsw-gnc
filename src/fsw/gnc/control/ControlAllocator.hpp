#pragma once

#include <Eigen/Dense>
#include <vector>
#include "common/types.hpp"
#include "common/logger.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief Reaction Wheel Control Allocator using Pseudo-Inverse.
 * 
 * Maps desired 3D spacecraft body torques to individual reaction wheel torques.
 * Supports a redundant 4-wheel pyramid configuration and dynamic reconfiguration
 * by zeroing out columns of failed wheels.
 */
class ControlAllocator {
   public:
    ControlAllocator() {
        // Initialize default 4-wheel pyramid configuration:
        // Skew angle alpha = 45 degrees.
        double c = 0.7071067811865475; // cos(45)
        double s = 0.7071067811865475; // sin(45)

        // w1 = [c, 0, s]
        // w2 = [0, c, s]
        // w3 = [-c, 0, s]
        // w4 = [0, -c, s]
        A_nominal_.resize(3, 4);
        A_nominal_ <<  c,  0.0, -c,  0.0,
                      0.0,  c,  0.0, -c,
                       s,   s,   s,   s;

        wheel_healthy_ = {true, true, true, true};
    }

    /**
     * @brief Set the health status of a specific wheel
     */
    void setWheelHealth(size_t index, bool healthy) {
        if (index < wheel_healthy_.size()) {
            if (wheel_healthy_[index] != healthy) {
                common::LogWarning("[Allocator] Wheel {} health status changed to {}", index, healthy ? "HEALTHY" : "FAILED");
                wheel_healthy_[index] = healthy;
            }
        }
    }

    /**
     * @brief Get the health status of a specific wheel
     */
    bool getWheelHealth(size_t index) const {
        if (index < wheel_healthy_.size()) {
            return wheel_healthy_[index];
        }
        return false;
    }

    /**
     * @brief Allocate 3D body torque command to reaction wheels
     * 
     * @param body_torque Desired control torque on body [Nm]
     * @param allocated_torques Output vector of individual wheel commands [Nm]
     * @return true if allocation succeeded, false if rank is deficient (unable to control 3D)
     */
    bool allocate(const common::Vector3& body_torque, std::vector<double>& allocated_torques) {
        allocated_torques.assign(4, 0.0);

        // Build reconfigured distribution matrix
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 4);
        for (int i = 0; i < 4; ++i) {
            if (wheel_healthy_[i]) {
                A.col(i) = A_nominal_.col(i);
            }
        }

        // Compute A * A^T
        common::Matrix3 AAT = A * A.transpose();
        double det = AAT.determinant();

        // Check if configuration is rank deficient (requires 3 independent axes)
        if (std::abs(det) < 1e-7) {
            common::LogError("[Allocator] Rank deficient wheel configuration (det = {:.3e}). Cannot allocate torque.", det);
            return false;
        }

        // Right pseudo-inverse: A^# = A^T * (A * A^T)^-1
        // Motor torque T_w = - A^# * T_body
        Eigen::VectorXd u = - A.transpose() * AAT.inverse() * body_torque;

        for (int i = 0; i < 4; ++i) {
            if (wheel_healthy_[i]) {
                allocated_torques[i] = u(i);
            } else {
                allocated_torques[i] = 0.0;
            }
        }

        return true;
    }

   private:
    Eigen::MatrixXd A_nominal_;
    std::vector<bool> wheel_healthy_;
};

} // namespace control
} // namespace gnc
} // namespace fsw
