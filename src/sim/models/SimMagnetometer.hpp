#pragma once
#include "hal/interfaces/IMagnetometer.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/dynamics/Orbit.hpp"

namespace sim {
namespace models {

/**
 * @brief Simulated Magnetometer
 */
class SimMagnetometer : public hal::IMagnetometer {
public:
    /**
     * @brief Constructor
     * @param body Reference to Rigid Body (for attitude)
     * @param orbit Reference to Orbit (for position)
     */
    SimMagnetometer(const dynamics::RigidBody& body, const dynamics::Orbit& orbit);

    common::Vector3 read() override;

private:
    const dynamics::RigidBody& body_;
    const dynamics::Orbit& orbit_;

    // Compute Earth's magnetic field in Inertial (ECI) frame using simple Dipole model
    common::Vector3 computeDipoleField(const common::Vector3& pos_eci) const;
};

} // namespace models
} // namespace sim
