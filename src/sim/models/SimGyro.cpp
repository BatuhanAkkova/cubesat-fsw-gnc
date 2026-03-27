#include "SimGyro.hpp"

#include <chrono>

namespace sim {
namespace models {

SimGyro::SimGyro(const dynamics::RigidBody& body, const Config& config)
    : body_(body),
      config_(config),
      gen_(std::chrono::system_clock::now().time_since_epoch().count()),
      dist_(0.0, 1.0) {}  // Initialize with 1.0, use config.noise_std during read

common::Vector3 SimGyro::read() {
    if (is_dead_) return common::Vector3::Zero();

    common::Vector3 true_omega = body_.getAngularVelocity();

    common::Vector3 measured = (true_omega + config_.bias) * scaling_factor_;

    // Add noise
    if (config_.noise_std > 0.0) {
        measured.x() += dist_(gen_) * config_.noise_std;
        measured.y() += dist_(gen_) * config_.noise_std;
        measured.z() += dist_(gen_) * config_.noise_std;
    }

    return measured;
}

}  // namespace models
}  // namespace sim
