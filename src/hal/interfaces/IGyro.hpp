#pragma once
#include "common/types.hpp"

namespace hal {

/**
 * @brief Interface for a Gyroscope sensor.
 */
class IGyro {
public:
    virtual ~IGyro() = default;

    /**
     * @brief Read the current angular velocity from the gyro.
     * @return Angular velocity vector in [rad/s] in the body frame.
     */
    virtual common::Vector3 read() = 0;
};

} // namespace hal
