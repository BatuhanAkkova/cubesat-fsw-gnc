#include "WheelDesaturation.hpp"
#include <cmath>
#include <algorithm>

namespace fsw {
namespace gnc {
namespace control {

WheelDesaturation::WheelDesaturation(const Config& config)
    : config_(config),
      state_(State::IDLE),
      max_momentum_(0.0),
      total_momentum_(0.0),
      time_in_state_(0.0) {
}

common::Vector3 WheelDesaturation::update(const std::vector<std::shared_ptr<hal::IRW>>& wheels,
                                          const common::Vector3& b_field_body,
                                          double dt) {
    // Update time in state
    time_in_state_ += dt;

    // Compute current momentum
    common::Vector3 h_wheels = computeTotalMomentum(wheels);
    total_momentum_ = h_wheels.norm();

    // Get maximum momentum if not initialized
    if (max_momentum_ <= 0.0) {
        max_momentum_ = computeMaxMomentum(wheels);
    }

    // Update state machine
    updateStateMachine(dt);

    // Determine if desaturation is active
    common::Vector3 dipole_cmd = common::Vector3::Zero();

    if (state_ == State::DESATURATING) {
        // Check if B-field is strong enough
        double b_magnitude = b_field_body.norm();
        if (b_magnitude > config_.min_b_field_magnitude) {
            // Compute B-cross-H control law
            dipole_cmd = computeBCrossH(h_wheels, b_field_body);
        }
    }

    return dipole_cmd;
}

void WheelDesaturation::reset() {
    state_ = State::IDLE;
    time_in_state_ = 0.0;
}

common::Vector3 WheelDesaturation::computeTotalMomentum(
        const std::vector<std::shared_ptr<hal::IRW>>& wheels) const {
    // For simplicity, assume wheels are aligned with body axes [X, Y, Z]
    common::Vector3 h_total = common::Vector3::Zero();

    if (wheels.size() >= 3) {
        h_total.x() = wheels[0]->getAngularMomentum();
        h_total.y() = wheels[1]->getAngularMomentum();
        h_total.z() = wheels[2]->getAngularMomentum();
    }

    return h_total;
}

double WheelDesaturation::computeMaxMomentum(
        const std::vector<std::shared_ptr<hal::IRW>>& wheels) const {
    if (wheels.empty()) {
        return 0.0;
    }

    // Compute vector of max momentum per wheel
    common::Vector3 h_max = common::Vector3::Zero();

    if (wheels.size() >= 3) {
        h_max.x() = wheels[0]->getMaxMomentum();
        h_max.y() = wheels[1]->getMaxMomentum();
        h_max.z() = wheels[2]->getMaxMomentum();
    }

    // Total max momentum is the norm of the max momentum vector
    return h_max.norm();
}

void WheelDesaturation::updateStateMachine(double dt) {
    // Calculate momentum ratio
    double momentum_ratio = (max_momentum_ > 0.0) ? (total_momentum_ / max_momentum_) : 0.0;

    switch (state_) {
        case State::IDLE:
            // Check if we need to start desaturation
            if (momentum_ratio > config_.momentum_threshold_ratio) {
                state_ = State::DESATURATING;
                time_in_state_ = 0.0;
            }
            break;

        case State::DESATURATING:
            // Check exit conditions
            if (momentum_ratio < config_.momentum_threshold_ratio * 0.7) {
                // Successfully desaturated (below 70% of threshold)
                state_ = State::COOLDOWN;
                time_in_state_ = 0.0;
            } else if (time_in_state_ > config_.max_desat_duration) {
                // Timeout - give up and cooldown
                state_ = State::COOLDOWN;
                time_in_state_ = 0.0;
            }
            break;

        case State::COOLDOWN:
            // Wait for minimum time before allowing another desaturation
            if (time_in_state_ > config_.min_time_between_desat) {
                state_ = State::IDLE;
                time_in_state_ = 0.0;
            }
            break;
    }
}

common::Vector3 WheelDesaturation::computeBCrossH(const common::Vector3& h_wheels,
                                                  const common::Vector3& b_field) const {
    // B-cross-H control law: M_cmd = -k * (h × B) × B / |B|^2
    // This produces a dipole moment perpendicular to B that reduces h along B
    
    double b_magnitude_sq = b_field.squaredNorm();
    
    if (b_magnitude_sq < config_.min_b_field_magnitude * config_.min_b_field_magnitude) {
        return common::Vector3::Zero();
    }

    // Compute h × B
    common::Vector3 h_cross_B = h_wheels.cross(b_field);
    
    // Compute (h × B) × B / |B|^2
    common::Vector3 h_cross_B_cross_B = h_cross_B.cross(b_field) / b_magnitude_sq;
    
    // Apply gain and negative sign
    // Negative sign because we want to reduce momentum (torque opposes h)
    common::Vector3 dipole_cmd = -config_.desat_gain * h_cross_B_cross_B;
    
    return dipole_cmd;
}

} // namespace control
} // namespace gnc
} // namespace fsw
