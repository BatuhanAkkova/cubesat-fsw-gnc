#pragma once

#include "hal/interfaces/IRW.hpp"

namespace sim {

/**
 * @brief Simulated Reaction Wheel model.
 * 
 * Models first-order dynamics of a reaction wheel:
 * J_w * dw/dt = tau_motor - tau_friction
 */
class SimRW : public hal::IRW {
public:
    struct Config {
        double inertia;         // kg*m^2
        double max_torque;      // Nm
        double max_momentum;    // Nms (implies max speed = max_momentum / inertia)
        double friction_coeff;  // Nms/rad (viscous friction) - optional
        double initial_speed;   // rad/s
    };

    SimRW(const Config& config);

    // IRW Interface
    void setTorqueCommand(double torque_nm) override;
    double getSpeed() const override;
    double getAngularMomentum() const override;
    double getMaxTorque() const override;
    double getMaxMomentum() const override;

    // Simulation Interface
    /**
     * @brief Step the wheel dynamics.
     * 
     * @param dt Time step in seconds.
     * @return double Torque EXERTED ON THE BODY (reaction torque). 
     *                Note: Torque on Body = - (Torque on Wheel).
     *                However, usually physics engine wants the torque applied to the body.
     *                If motor accelerates wheel (+), it exerts (-) torque on body.
     */
    double step(double dt);

    /**
     * @brief Set the physical speed directly (e.g. initialization).
     */
    void setSpeed(double speed_rad_s);

private:
    Config config_;
    double current_speed_;      // rad/s
    double commanded_torque_;   // Nm
};

} // namespace sim
