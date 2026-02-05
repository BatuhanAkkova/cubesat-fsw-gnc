#pragma once
#include "common/types.hpp"

namespace sim {
namespace dynamics {

class Orbit {
public:
    /**
     * @brief Configuration for orbit propagator
     */
    struct Config {
        bool enable_j2 = true;  // Enable J2 oblateness perturbation
    };

    /**
     * @brief Constructor
     * @param init_pos Initial position [m] in ECI
     * @param init_vel Initial velocity [m/s] in ECI
     * @param config Orbit propagator configuration
     */
    Orbit(const common::Vector3& init_pos, const common::Vector3& init_vel, 
          const Config& config = Config());

    void step(double dt);

    common::Vector3 getPosition() const;
    common::Vector3 getVelocity() const;

    /**
     * @brief Compute orbital elements from current state
     * @return Semi-major axis [m]
     */
    double getSemiMajorAxis() const;

    /**
     * @brief Compute orbital eccentricity
     * @return Eccentricity (dimensionless)
     */
    double getEccentricity() const;

    /**
     * @brief Compute orbital inclination
     * @return Inclination [rad]
     */
    double getInclination() const;

    /**
     * @brief Compute Right Ascension of Ascending Node
     * @return RAAN [rad]
     */
    double getRaan() const;

    /**
     * @brief Compute Argument of Perigee
     * @return Argument of perigee [rad]
     */
    double getArgumentOfPerigee() const;

private:
    // State: [rx, ry, rz, vx, vy, vz] (6 elements)
    common::VectorX state_;
    Config config_;

    common::VectorX dynamics(double t, const common::VectorX& y);
    
    /**
     * @brief Compute J2 perturbation acceleration
     * @param pos Position vector [m] in ECI
     * @return J2 acceleration [m/s^2]
     */
    common::Vector3 computeJ2Acceleration(const common::Vector3& pos) const;
};

} // namespace dynamics
} // namespace sim
