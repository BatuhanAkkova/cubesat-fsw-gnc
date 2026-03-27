#pragma once

#include <cmath>

#include "common/types.hpp"
#include "fsw/gnc/control/PID.hpp"
#include "fsw/gnc/interfaces/IController.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief 3-Axis Attitude Controller with gain scheduling and rate limiting.
 *
 * Takes current attitude (quaternion) and target attitude, computes error,
 * and outputs torque commands for each axis using PID controllers.
 * Gain scheduling based on attitude error magnitude
 * Rate limiting to prevent rapid acceleration
 * Proper quaternion error (no small-angle approximation)
 * Direct rate feedback for better damping
 */
class AttitudeController : public interfaces::IController {
   public:
    struct Config {
        // Nominal PID gains (for small errors < 10 deg)
        PID::Config nominal_pid;

        // Large error PID gains (for errors > 30 deg)
        PID::Config large_error_pid;

        // Rate limiting
        double max_torque_rate;    // Maximum torque change rate [Nm/s]
        double max_angular_accel;  // Maximum angular acceleration [rad/s^2]

        // Gain scheduling thresholds
        double small_error_threshold;  // rad, below this use nominal gains
        double large_error_threshold;  // rad, above this use large-error gains

        // Direct rate feedback gain
        double rate_feedback_gain;  // Damping term: -k_rate * omega

        Config()
            : max_torque_rate(0.5),          // 0.5 Nm/s
              max_angular_accel(0.1),        // 0.1 rad/s^2
              small_error_threshold(0.175),  // 10 degrees
              large_error_threshold(0.524),  // 30 degrees
              rate_feedback_gain(0.05) {}
    };

    AttitudeController(const Config& config)
        : config_(config),
          pid_roll_(config.nominal_pid),
          pid_pitch_(config.nominal_pid),
          pid_yaw_(config.nominal_pid),
          last_torque_cmd_(common::Vector3::Zero()) {}

    /**
     * @brief Update control torque with gain scheduling and rate limiting.
     *
     * @param sensors Latest sensor measurements.
     * @param state Latest estimated state.
     * @param target Guidance target.
     * @param dt Time step [seconds]
     * @return common::Vector3 Commanded torque in Body frame [Nm]
     */
    common::Vector3 update(const common::SensorData& sensors, const common::State& state_curr,
                           const common::GuidanceTarget& target, double dt) override {
        // 1. Calculate Attitude Error (Quaternion)
        // q_err = q_curr_inv * q_target (Error from current to target in Body frame)
        common::Quaternion q_err = state_curr.q.inverse() * target.q;

        // Ensure q_err is the "shortest path"
        if (q_err.w() < 0) {
            q_err.coeffs() *= -1.0;
        }

        // 2. Compute error angle for gain scheduling
        double error_angle = 2.0 * std::acos(std::min(1.0, std::abs(q_err.w())));

        // 3. Extract error axis-angle representation
        common::Vector3 error_vec;
        if (error_angle < 1e-6) {
            error_vec = 2.0 * q_err.vec();
        } else {
            double sin_half = std::sin(error_angle / 2.0);
            if (std::abs(sin_half) > 1e-9) {
                error_vec = (error_angle / sin_half) * q_err.vec();
            } else {
                error_vec = 2.0 * q_err.vec();
            }
        }

        // 4. Apply gain scheduling
        double gain_factor = computeGainFactor(error_angle);
        PID::Config scheduled_config = config_.nominal_pid;
        if (gain_factor < 1.0) {
            double alpha = gain_factor;
            scheduled_config.kp = alpha * config_.nominal_pid.kp + (1.0 - alpha) * config_.large_error_pid.kp;
            scheduled_config.kd = alpha * config_.nominal_pid.kd + (1.0 - alpha) * config_.large_error_pid.kd;
        }

        pid_roll_.setGains(scheduled_config.kp, scheduled_config.ki, scheduled_config.kd);
        pid_pitch_.setGains(scheduled_config.kp, scheduled_config.ki, scheduled_config.kd);
        pid_yaw_.setGains(scheduled_config.kp, scheduled_config.ki, scheduled_config.kd);

        // 5. Compute PID torques
        double tx = pid_roll_.calculate(error_vec.x(), dt);
        double ty = pid_pitch_.calculate(error_vec.y(), dt);
        double tz = pid_yaw_.calculate(error_vec.z(), dt);

        common::Vector3 torque_pid(tx, ty, tz);

        // 6. Add direct rate feedback for better damping
        // Note: target.w is the target rates, state_curr.w is current estimated rate
        common::Vector3 torque_damping = -config_.rate_feedback_gain * (state_curr.w - target.w);

        common::Vector3 torque_cmd = torque_pid + torque_damping;

        // 7. Apply rate limiting
        torque_cmd = applyRateLimiting(torque_cmd, dt);

        return torque_cmd;
    }

    void reset() override {
        pid_roll_.reset();
        pid_pitch_.reset();
        pid_yaw_.reset();
        last_torque_cmd_ = common::Vector3::Zero();
    }

    /**
     * @brief Set PID gains for all axes.
     * For simplicity, set the same gains for all three axes.
     */
    void setGains(double kp, double ki, double kd, bool is_nominal = true) {
        if (is_nominal) {
            config_.nominal_pid.kp = kp;
            config_.nominal_pid.ki = ki;
            config_.nominal_pid.kd = kd;
            pid_roll_.setGains(kp, ki, kd);
            pid_pitch_.setGains(kp, ki, kd);
            pid_yaw_.setGains(kp, ki, kd);
        } else {
            config_.large_error_pid.kp = kp;
            config_.large_error_pid.ki = ki;
            config_.large_error_pid.kd = kd;
        }
    }

    Config& getConfig() {
        return config_;
    }

   private:
    /**
     * @brief Compute gain scheduling factor based on error magnitude.
     * @return 1.0 for small errors, 0.0 for large errors, interpolated in between
     */
    double computeGainFactor(double error_angle) const {
        if (error_angle <= config_.small_error_threshold) {
            return 1.0;  // Use nominal gains
        } else if (error_angle >= config_.large_error_threshold) {
            return 0.0;  // Use large-error gains
        } else {
            // Linear interpolation
            double range = config_.large_error_threshold - config_.small_error_threshold;
            return 1.0 - (error_angle - config_.small_error_threshold) / range;
        }
    }

    /**
     * @brief Apply rate limiting to torque command.
     */
    common::Vector3 applyRateLimiting(const common::Vector3& desired_torque, double dt) {
        if (dt <= 0.0) return last_torque_cmd_;

        common::Vector3 torque_delta = desired_torque - last_torque_cmd_;
        double delta_norm = torque_delta.norm();

        // Limit rate of change
        double max_delta = config_.max_torque_rate * dt;
        if (delta_norm > max_delta) {
            torque_delta = (max_delta / delta_norm) * torque_delta;
        }

        common::Vector3 limited_torque = last_torque_cmd_ + torque_delta;
        last_torque_cmd_ = limited_torque;

        return limited_torque;
    }

    Config config_;
    PID pid_roll_;
    PID pid_pitch_;
    PID pid_yaw_;
    common::Vector3 last_torque_cmd_;
};

}  // namespace control
}  // namespace gnc
}  // namespace fsw
