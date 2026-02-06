#include "SimStarTracker.hpp"
#include <chrono>

namespace sim {
namespace models {

SimStarTracker::SimStarTracker(const dynamics::RigidBody& body, double noise_std)
    : body_(body), noise_std_(noise_std), 
      gen_(std::chrono::system_clock::now().time_since_epoch().count()), 
      dist_(0.0, 1.0) {}

common::Quaternion SimStarTracker::getOrientation() const {
    // Ground truth: Body -> Inertial
    common::Quaternion q_bi = body_.getAttitude();
    
    // We want Inertial -> Body
    common::Quaternion q_ib = q_bi.inverse();

    if (noise_std_ > 0.0) {
        // Generate random small rotation vector
        double dx = dist_(gen_) * noise_std_;
        double dy = dist_(gen_) * noise_std_;
        double dz = dist_(gen_) * noise_std_;

        // Small angle quaternion approximation: [1, dx/2, dy/2, dz/2]
        // Note: Eigen quaternion constructor is (w, x, y, z)
        common::Quaternion noise(1.0, dx * 0.5, dy * 0.5, dz * 0.5);
        noise.normalize();

        // Apply noise in Body frame (since sensor is attached to body)
        // q_meas = noise * q_true
        return noise * q_ib;
    }

    return q_ib;
}

} // namespace models
} // namespace sim
