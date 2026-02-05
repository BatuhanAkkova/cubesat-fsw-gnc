#include "SimGyro.hpp"

namespace sim {
namespace models {

SimGyro::SimGyro(const dynamics::RigidBody& body, const Config& config)
    : body_(body), config_(config), gen_(std::random_device{}()), dist_(0.0, config.noise_std) {}

common::Vector3 SimGyro::read() {
    common::Vector3 omega = body_.getAngularVelocity();
    
    // Add bias
    omega += config_.bias;
    
    // Add noise
    if (config_.noise_std > 0.0) {
        omega.x() += dist_(gen_);
        omega.y() += dist_(gen_);
        omega.z() += dist_(gen_);
    }
    
    return omega;
}

} // namespace models
} // namespace sim
