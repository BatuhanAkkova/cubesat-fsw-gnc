#pragma once
#include "common/types.hpp"

namespace common {

/**
 * @brief Struct to hold all sensor measurements for a mission step.
 */
struct SensorData {
    common::Vector3 mag_body;       // Tesla
    common::Vector3 gyro_body;      // rad/s
    common::Quaternion q_measured;  // Attitude estimate (from Star Tracker or MEKF)
    common::Vector3 sun_body;       // Sun vector in body frame
};

} // namespace common
