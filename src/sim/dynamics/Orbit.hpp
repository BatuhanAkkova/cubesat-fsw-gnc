#pragma once
#include "common/types.hpp"

namespace sim {
namespace dynamics {

class Orbit {
public:
    /**
     * @brief Constructor
     * @param init_pos Initial position [m] in ECI
     * @param init_vel Initial velocity [m/s] in ECI
     */
    Orbit(const common::Vector3& init_pos, const common::Vector3& init_vel);

    void step(double dt);

    common::Vector3 getPosition() const;
    common::Vector3 getVelocity() const;

private:
    // State: [rx, ry, rz, vx, vy, vz] (6 elements)
    common::VectorX state_;

    common::VectorX dynamics(double t, const common::VectorX& y);
};

} // namespace dynamics
} // namespace sim
