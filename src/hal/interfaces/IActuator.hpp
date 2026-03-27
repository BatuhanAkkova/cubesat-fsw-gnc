#pragma once
#include <string>

namespace hal {

/**
 * @brief Base interface for all actuators (Torquers, Reaction Wheels, etc.).
 */
class IActuator {
public:
    virtual ~IActuator() = default;

    /**
     * @brief Get the actuator name/alias.
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Stop the actuator (safety/reset).
     */
    virtual void stop() = 0;

    /**
     * @brief Check if the actuator is healthy.
     */
    virtual bool isHealthy() const { return true; }
};

} // namespace hal
