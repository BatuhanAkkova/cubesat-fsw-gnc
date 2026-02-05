#pragma once

#include "common/types.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief Linear Quadratic Regulator (LQR) for attitude control.
 * 
 * Computes control torque: u = -K * x
 * where x is the state vector [delta_theta (3x1), delta_omega (3x1)]^T
 * and K is the 3x6 gain matrix.
 */
class LQRController {
public:
    struct Config {
        common::MatrixX K; // Gain matrix (3 x 6)
        
        Config() : K(common::MatrixX::Zero(3, 6)) {}
        
        /**
         * @brief Initialize with diagonal weights (simplified)
         * @param q_rot weighting for attitude error
         * @param q_rate weighting for angular velocity
         */
        static Config Default(double q_rot = 1.0, double q_rate = 0.5) {
            Config cfg;
            cfg.K = common::MatrixX::Zero(3, 6);
            for (int i = 0; i < 3; ++i) {
                cfg.K(i, i) = q_rot;      // Proportional-like gain
                cfg.K(i, i + 3) = q_rate; // Derivative-like gain
            }
            return cfg;
        }
    };

    LQRController(const Config& config) : config_(config) {}

    /**
     * @brief Compute LQR torque: u = -K * x
     * @param x State vector [delta_theta (3x1), delta_omega (3x1)]^T
     * @return common::Vector3 Commanded torque [Nm]
     */
    common::Vector3 computeTorque(const common::VectorX& x) const {
        if (x.size() != 6 || config_.K.cols() != 6 || config_.K.rows() != 3) {
            return common::Vector3::Zero();
        }
        return -config_.K * x;
    }

    /**
     * @brief Update regulator gains
     */
    void setGains(const common::MatrixX& K) {
        if (K.rows() == 3 && K.cols() == 6) {
            config_.K = K;
        }
    }

    const Config& getConfig() const { return config_; }

private:
    Config config_;
};

} // namespace control
} // namespace gnc
} // namespace fsw
