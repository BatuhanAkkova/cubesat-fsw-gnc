#include "SimStarTracker.hpp"

namespace sim {
namespace models {

SimStarTracker::SimStarTracker(const dynamics::RigidBody& body, double noise_std)
    : body_(body), noise_std_(noise_std), gen_(std::random_device{}()), dist_(0.0, noise_std) {}

common::Quaternion SimStarTracker::getOrientation() const {
    // Ground truth: Body -> Inertial
    common::Quaternion q_bi = body_.getAttitude();
    
    // We want Inertial -> Body
    common::Quaternion q_ib = q_bi.inverse();

    if (noise_std_ > 0.0) {
        // Generate random small rotation vector
        double dx = dist_(gen_);
        double dy = dist_(gen_);
        double dz = dist_(gen_);

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
