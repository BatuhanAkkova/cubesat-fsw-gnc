#pragma once

#include "hal/interfaces/IRW.hpp"

namespace sim {

/**
 * @brief Configuration for reaction wheel model.
 */
struct SimRWConfig {
    double inertia;         // kg*m^2
    double max_torque;      // Nm
    double max_momentum;    // Nms (implies max speed = max_momentum / inertia)
    double friction_coeff;  // Nms/rad (viscous friction) - optional
    double initial_speed;   // rad/s
};

/**
 * @brief Simulated Reaction Wheel model.
 *
 * Models first-order dynamics of a reaction wheel:
 * J_w * dw/dt = tau_motor - tau_friction
 */
class SimRW : public hal::IRW {
   public:
    using Config = SimRWConfig;

    SimRW(const Config& config);

    // IRW Interface
    void setTorqueCommand(double torque_nm) override;
    double getSpeed() const override;
    double getAngularMomentum() const override;
    double getMaxTorque() const override;
    double getMaxMomentum() const override;
    void stop() override {
        commanded_torque_ = 0.0;
    }

    // Simulation Interface
    /**
     * @brief Step the wheel dynamics.
     *
     * @param dt Time step in seconds.
     * @return double Torque exerted on the body (reaction torque).
     *                Note: Torque on Body = - (Torque on Wheel).
     */
    double step(double dt);

    /**
     * @brief Set the physical speed directly (e.g. initialization).
     */
    void setSpeed(double speed_rad_s);

    void injectFailure_Dead() {
        is_dead_ = true;
    }
    void setEfficiency(double efficiency) {
        efficiency_ = efficiency;
    }

   private:
    Config config_;
    double current_speed_;     // rad/s
    double commanded_torque_;  // Nm
    bool is_dead_ = false;
    double efficiency_ = 1.0;
};

}  // namespace sim
