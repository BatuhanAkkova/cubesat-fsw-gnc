#pragma once
#include <random>

#include "hal/interfaces/IMagnetometer.hpp"
#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"

namespace sim {
namespace models {

/**
 * @brief Simulated Magnetometer
 */
/**
 * @brief Configuration for magnetic field model
 */
struct SimMagnetometerConfig {
    bool enable_earth_rotation;  // Enable time-varying field due to Earth rotation
    bool use_tilted_dipole;      // Use tilted dipole (11 deg offset)
    double noise_std;            // Standard deviation of noise [Tesla]
    common::Vector3 bias;        // Hard-iron bias [Tesla]

    SimMagnetometerConfig()
        : enable_earth_rotation(true), use_tilted_dipole(false), noise_std(0.0), bias(common::Vector3::Zero()) {}
};

class SimMagnetometer : public hal::IMagnetometer {
   public:
    using Config = SimMagnetometerConfig;

    /**
     * @brief Constructor
     * @param body Reference to Rigid Body (for attitude)
     * @param orbit Reference to Orbit (for position)
     * @param config Magnetometer configuration
     */
    SimMagnetometer(const dynamics::RigidBody& body, const dynamics::Orbit& orbit, const Config& config = Config());

    common::Vector3 read() override;

    /**
     * @brief Set simulation time (for Earth rotation)
     * @param time_sec Simulation time [seconds] since J2000 epoch
     */
    void setTime(double time_sec);

    void injectFailure_Dead() {
        is_dead_ = true;
    }
    void setScalingFactor(double factor) {
        scaling_factor_ = factor;
    }

   private:
    const dynamics::RigidBody& body_;
    const dynamics::Orbit& orbit_;
    Config config_;
    double sim_time_;  // Simulation time [s]

    bool is_dead_ = false;
    double scaling_factor_ = 1.0;

    mutable std::mt19937 gen_;
    mutable std::normal_distribution<double> dist_;

    // Compute Earth's magnetic field in Inertial (ECI) frame using dipole model
    common::Vector3 computeDipoleField(const common::Vector3& pos_eci, double time_sec) const;
};

}  // namespace models
}  // namespace sim
