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
     * @brief Configuration for magnetic field model
     */
    struct Config {
        bool enable_earth_rotation = true;  // Enable time-varying field due to Earth rotation
        bool use_tilted_dipole = false;     // Use tilted dipole (11 deg offset)
    };

    /**
     * @brief Constructor
     * @param body Reference to Rigid Body (for attitude)
     * @param orbit Reference to Orbit (for position)
     * @param config Magnetometer configuration
     */
    SimMagnetometer(const dynamics::RigidBody& body, const dynamics::Orbit& orbit,
                    const Config& config = Config());

    common::Vector3 read() override;

    /**
     * @brief Set simulation time (for Earth rotation)
     * @param time_sec Simulation time [seconds] since J2000 epoch
     */
    void setTime(double time_sec);

private:
    const dynamics::RigidBody& body_;
    const dynamics::Orbit& orbit_;
    Config config_;
    double sim_time_;  // Simulation time [s]

    // Compute Earth's magnetic field in Inertial (ECI) frame using dipole model
    common::Vector3 computeDipoleField(const common::Vector3& pos_eci, double time_sec) const;
};

} // namespace models
} // namespace sim
