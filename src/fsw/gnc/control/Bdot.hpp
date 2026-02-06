#pragma once
#include "common/types.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief B-Dot Magnetic Controller
 * 
 * Implements the B-Dot control law: M = -K * B_dot
 * Use to detumble the spacecraft.
 */
class Bdot {
public:
    /**
     * @brief Constructor
     * @param gain Controller gain [Am^2 * s / T] typically. 
     */
    explicit Bdot(double gain);

    /**
     * @brief Calculate control dipole moment
     * @param b_body_T Magnetic field measurement in Body frame [Tesla]
     * @param dt Time step since last update [seconds]
     * @return Commanded dipole moment [Am^2] in Body frame
     */
    common::Vector3 update(const common::Vector3& b_body_T, double dt);

    /**
     * @brief Reset the controller state (e.g. previous measurement)
     */
    void reset();

private:
    double gain_;
    common::Vector3 b_prev_;
    bool first_run_;
};

} // namespace control
} // namespace gnc
} // namespace fsw
