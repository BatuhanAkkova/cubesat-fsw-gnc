#pragma once

#include <algorithm>

#include "fsw/gnc/interfaces/IController.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief Discrete PID Controller with anti-windup.
 */
class PID {
   public:
    struct Config {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double limit = 1.0e12;              // Output limit (saturation)
        double anti_windup_limit = 1.0e12;  // I-term clamp
    };

    PID(const Config& config) : config_(config), integral_(0.0), last_error_(0.0), last_output_(0.0), first_run_(true) {}

    /**
     * @brief Compute PID output.
     *
     * @param error Current error (setpoint - feedback)
     * @param dt Sampling time in seconds
     * @return double Controller effort
     */
    double calculate(double error, double dt) {
        if (dt <= 0.0) return last_output_;

        // Proportional term
        double p_term = config_.kp * error;

        // Integral term with anti-windup
        integral_ += error * dt;
        double i_term = config_.ki * integral_;
        i_term = std::max(-config_.anti_windup_limit, std::min(config_.anti_windup_limit, i_term));

        // Update integral_ to match clamped i_term if ki != 0
        if (std::abs(config_.ki) > 1e-9) {
            integral_ = i_term / config_.ki;
        }

        // Derivative term
        double d_term = 0.0;
        if (!first_run_) {
            d_term = config_.kd * (error - last_error_) / dt;
        }
        first_run_ = false;

        // Total output with saturation
        double output = p_term + i_term + d_term;
        output = std::max(-config_.limit, std::min(config_.limit, output));

        last_error_ = error;
        last_output_ = output;
        return output;
    }

    void reset() {
        integral_ = 0.0;
        last_error_ = 0.0;
        first_run_ = true;
    }

    void setGains(double kp, double ki, double kd) {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    Config getConfig() const {
        return config_;
    }

   private:
    Config config_;
    double integral_;
    double last_error_;
    double last_output_;
    bool first_run_;
};

}  // namespace control
}  // namespace gnc
}  // namespace fsw
