#include "sim/models/SimRW.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

SimRW::SimRW(const Config& config) : config_(config), current_speed_(config.initial_speed), commanded_torque_(0.0) {}

void SimRW::setTorqueCommand(double torque_nm) {
    // Saturate command
    commanded_torque_ = std::max(-config_.max_torque, std::min(config_.max_torque, torque_nm));
}

double SimRW::getSpeed() const {
    return current_speed_;
}

double SimRW::getAngularMomentum() const {
    return config_.inertia * current_speed_;
}

double SimRW::getMaxTorque() const {
    return config_.max_torque;
}

double SimRW::getMaxMomentum() const {
    return config_.max_momentum;
}

void SimRW::setSpeed(double speed_rad_s) {
    current_speed_ = speed_rad_s;
}

double SimRW::step(double dt) {
    if (is_dead_) {
        commanded_torque_ = 0.0;
    }

    if (fault_type_ == RWFaultType::STUCK) {
        current_speed_ = stuck_speed_;
        return 0.0; // No reaction torque on body when stuck
    }

    // Apply efficiency to the commanded torque
    double actual_motor_torque = commanded_torque_ * efficiency_;

    // J_w * dw/dt = tau_motor - tau_friction
    double friction_torque = config_.friction_coeff * current_speed_;
    double net_torque = actual_motor_torque - friction_torque;

    // Check momentum saturation (speed limit)
    // If at max speed and trying to accelerate further, clamp torque
    double current_momentum = std::abs(getAngularMomentum());
    if (current_momentum >= config_.max_momentum || fault_type_ == RWFaultType::SATURATED) {
        // Force speed to max speed if injected saturated fault
        if (fault_type_ == RWFaultType::SATURATED) {
            double max_speed = config_.max_momentum / config_.inertia;
            if (current_speed_ >= 0.0) {
                current_speed_ = max_speed;
            } else {
                current_speed_ = -max_speed;
            }
        }
        // Clamping torque if accelerating in the saturated direction
        if (current_speed_ > 0 && net_torque > 0) {
            net_torque = 0.0;
        } else if (current_speed_ < 0 && net_torque < 0) {
            net_torque = 0.0;
        }
    }

    double angular_accel = net_torque / config_.inertia;
    current_speed_ += angular_accel * dt;

    return -net_torque;
}

}  // namespace sim
