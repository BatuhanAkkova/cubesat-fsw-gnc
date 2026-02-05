#include "sim/models/SimRW.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

SimRW::SimRW(const Config& config) 
    : config_(config), 
      current_speed_(config.initial_speed), 
      commanded_torque_(0.0) {
}

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
    // Simple Euler integration for wheel speed
    // J * dw/dt = T_motor - T_friction
    // T_friction assumption: proportional to speed (viscous)
    
    double friction_torque = config_.friction_coeff * current_speed_;
    double net_torque_on_wheel = commanded_torque_ - friction_torque;

    // Check momentum saturation (speed limit)
    // If at max speed and trying to accelerate further, clamp torque
    double current_momentum = std::abs(getAngularMomentum());
    if (current_momentum >= config_.max_momentum) {
        // If speed is + and torque is +, clamp
        if (current_speed_ > 0 && net_torque_on_wheel > 0) {
            net_torque_on_wheel = 0; 
        }
        // If speed is - and torque is -, clamp
        else if (current_speed_ < 0 && net_torque_on_wheel < 0) {
            net_torque_on_wheel = 0;
        }
    }

    double angular_accel = net_torque_on_wheel / config_.inertia;
    current_speed_ += angular_accel * dt;
    
    return -net_torque_on_wheel; 
}

} // namespace sim
