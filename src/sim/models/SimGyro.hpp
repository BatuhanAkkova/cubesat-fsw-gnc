#pragma once
#include "hal/interfaces/IGyro.hpp"
#include "sim/dynamics/RigidBody.hpp"

namespace sim {
namespace models {

/**
 * @brief Simulated Gyroscope
 */
class SimGyro : public hal::IGyro {
public:
    /**
     * @brief Constructor
     * @param body Reference to the simulated rigid body dynamics
     */
    explicit SimGyro(const dynamics::RigidBody& body);

    common::Vector3 read() override;

private:
    const dynamics::RigidBody& body_;
    // TODO: Add bias, random walk, noise params
};

} // namespace models
} // namespace sim
