#pragma once
#include <string>

namespace hal {

/**
 * @brief Base interface for all sensors.
 */
class ISensor {
public:
    virtual ~ISensor() = default;

    /**
     * @brief Get the sensor name/alias.
     */
    virtual std::string getName() const = 0;

    /**
     * @brief Check if the sensor is healthy.
     */
    virtual bool isHealthy() const { return true; }
};

} // namespace hal
