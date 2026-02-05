#pragma once

namespace hal {

/**
 * @brief Interface for a Reaction Wheel actuator.
 */
class IRW {
public:
    virtual ~IRW() = default;

    /**
     * @brief Command a torque to the reaction wheel.
     * @param torque_Nm Desired torque in [Nm].
     */
    virtual void setTorque(double torque_Nm) = 0;

    /**
     * @brief Get the current speed of the reaction wheel.
     * @return Speed in [rad/s].
     */
    virtual double getSpeed() const = 0;
};

} // namespace hal
