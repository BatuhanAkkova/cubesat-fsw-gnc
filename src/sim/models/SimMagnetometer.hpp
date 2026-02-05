#pragma once
#include "hal/interfaces/IMagnetometer.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/dynamics/Orbit.hpp"
#include <random>

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
        double noise_std = 0.0;             // Standard deviation of noise [Tesla]
        common::Vector3 bias = common::Vector3::Zero(); // Hard-iron bias [Tesla]
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

    mutable std::mt19937 gen_;
    mutable std::normal_distribution<double> dist_;

    // Compute Earth's magnetic field in Inertial (ECI) frame using dipole model
    common::Vector3 computeDipoleField(const common::Vector3& pos_eci, double time_sec) const;
};

} // namespace models
} // namespace sim
