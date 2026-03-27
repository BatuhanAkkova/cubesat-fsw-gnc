#pragma once
#include <string>

#include "common/types.hpp"
#include "hal/interfaces/IActuator.hpp"

namespace hal {

/**
 * @brief Interface for a Reaction Wheel (RW).
 *
 * Provides methods to command torque and read speed/momentum.
 */
class IRW : public IActuator {
   public:
    virtual ~IRW() = default;

    std::string getName() const override {
        return "ReactionWheel";
    }

    /**
     * @brief Set the torque command for the wheel.
     *
     * @param torque_nm Torque in Newton-meters. Positive torque accelerates the wheel.
     */
    virtual void setTorqueCommand(double torque_nm) = 0;

    /**
     * @brief Get the current speed of the wheel.
     *
     * @return double Angular velocity in rad/s.
     */
    virtual double getSpeed() const = 0;

    /**
     * @brief Get the angular momentum of the wheel.
     *
     * @return double Angular momentum in N*m*s (kg*m^2/s).
     */
    virtual double getAngularMomentum() const = 0;

    /**
     * @brief Get the maximum torque capability of the wheel.
     *
     * @return double Max torque in Nm.
     */
    virtual double getMaxTorque() const = 0;

    /**
     * @brief Get the maximum momentum storage characteristic.
     *
     * @return double Max momentum in Nms.
     */
    virtual double getMaxMomentum() const = 0;
};

}  // namespace hal
