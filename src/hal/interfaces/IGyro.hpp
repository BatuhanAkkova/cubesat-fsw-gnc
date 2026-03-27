#pragma once
#include "common/types.hpp"
#include "hal/interfaces/ISensor.hpp"

namespace hal {

/**
 * @brief Interface for a Gyroscope sensor.
 */
class IGyro : public ISensor {
public:
    virtual ~IGyro() = default;

    std::string getName() const override { return "Gyroscope"; }

    /**
     * @brief Read the current angular velocity from the gyro.
     * @return Angular velocity vector in [rad/s] in the body frame.
     */
    virtual common::Vector3 read() = 0;
};

} // namespace hal
