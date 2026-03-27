#pragma once
#include "common/types.hpp"
#include "hal/interfaces/ISensor.hpp"

namespace hal {

/**
 * @brief Interface for a Magnetometer sensor.
 */
class IMagnetometer : public ISensor {
public:
    virtual ~IMagnetometer() = default;

    std::string getName() const override { return "Magnetometer"; }

    /**
     * @brief Read the current magnetic field vector.
     * @return Magnetic field vector in [Tesla] (or appropriate unit) in the body frame.
     */
    virtual common::Vector3 read() = 0;
};

} // namespace hal
