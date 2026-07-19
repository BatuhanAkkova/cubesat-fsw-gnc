#pragma once
#include <vector>
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

    // Reaction wheel telemetry
    std::vector<double> rw_speeds;       // rad/s
    std::vector<double> rw_torques_prev; // Nm
};

}  // namespace common
