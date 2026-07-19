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

    enum class RWFaultType { NONE, STUCK, SATURATED, DEGRADED_TORQUE };

    void injectFailure_Dead() {
        is_dead_ = true;
    }
    void setEfficiency(double efficiency) {
        efficiency_ = efficiency;
    }

    void injectFault_Stuck(double speed_rad_s) {
        fault_type_ = RWFaultType::STUCK;
        stuck_speed_ = speed_rad_s;
        current_speed_ = speed_rad_s;
    }

    void injectFault_Saturated() {
        fault_type_ = RWFaultType::SATURATED;
        current_speed_ = config_.max_momentum / config_.inertia;
    }

    void injectFault_DegradedTorque(double efficiency) {
        fault_type_ = RWFaultType::DEGRADED_TORQUE;
        efficiency_ = efficiency;
    }

    void resetFaults() {
        is_dead_ = false;
        fault_type_ = RWFaultType::NONE;
        efficiency_ = 1.0;
        stuck_speed_ = 0.0;
    }

    RWFaultType getFaultType() const {
        return fault_type_;
    }

   private:
    Config config_;
    double current_speed_;     // rad/s
    double commanded_torque_;  // Nm
    bool is_dead_ = false;
    double efficiency_ = 1.0;
    RWFaultType fault_type_ = RWFaultType::NONE;
    double stuck_speed_ = 0.0;
};

}  // namespace sim
