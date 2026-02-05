#include "SimGyro.hpp"

namespace sim {
namespace models {

SimGyro::SimGyro(const dynamics::RigidBody& body)
    : body_(body) {}

common::Vector3 SimGyro::read() {
    // In the future, add sensor errors (bias, noise) here.
    return body_.getAngularVelocity();
}

} // namespace models
} // namespace sim
