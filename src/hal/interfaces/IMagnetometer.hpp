#pragma once
#include "common/types.hpp"

namespace hal {

/**
 * @brief Interface for a Magnetometer sensor.
 */
class IMagnetometer {
public:
    virtual ~IMagnetometer() = default;

    /**
     * @brief Read the current magnetic field vector.
     * @return Magnetic field vector in [Tesla] (or appropriate unit) in the body frame.
     */
    virtual common::Vector3 read() = 0;
};

} // namespace hal
