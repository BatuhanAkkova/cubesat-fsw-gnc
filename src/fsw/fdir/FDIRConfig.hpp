#pragma once

namespace fsw {
namespace fdir {

/**
 * @brief Configuration for Gyroscope health monitoring
 */
struct GyroHealthConfig {
    // Stuck detection parameters
    double stuck_tolerance = 1e-6;        // rad/s - minimum change to consider sensor active
    int stuck_sample_count = 50;          // samples - detect as stuck after this many static readings
    
    // Range detection parameters
    double max_rate = 10.0;               // rad/s - maximum physically reasonable angular rate
    
    // Rapid change detection parameters
    double max_rate_change = 1.0;         // rad/s^2 - maximum angular acceleration
    double dt = 0.1;                      // seconds - expected sample period
};

/**
 * @brief Configuration for Magnetometer health monitoring
 */
struct MagnetometerHealthConfig {
    // Stuck detection parameters
    double stuck_tolerance = 1e-9;        // Tesla - minimum change to consider sensor active
    int stuck_sample_count = 50;          // samples
    
    // Range detection parameters  
    double min_field = 1e-6;              // Tesla - minimum Earth field magnitude (~20 µT at poles)
    double max_field = 1e-4;              // Tesla - maximum Earth field magnitude (~65 µT)
    
    // Rapid change detection parameters
    double max_field_change = 1e-5;       // Tesla/s - max rate of change in LEO
    double dt = 0.1;                      // seconds - expected sample period
};

/**
 * @brief Configuration for Star Tracker health monitoring
 */
struct StarTrackerHealthConfig {
    // Stuck detection parameters
    double stuck_tolerance = 1e-6;        // rad - minimum quaternion angle change
    int stuck_sample_count = 30;          // samples - slower update rate typically
    
    // Range detection parameters
    // Quaternions are always normalized, so no explicit range check needed
    // But we can check if quaternion norm deviates from 1.0
    double quat_norm_tolerance = 0.01;    // dimensionless - acceptable deviation from unit norm
    
    // Rapid change detection parameters
    double max_angle_change = 0.1;        // rad - maximum attitude change between measurements
    double dt = 1.0;                      // seconds - star tracker typically slower update
};

/**
 * @brief Health status enumeration
 */
enum class HealthStatus {
    HEALTHY,    // Sensor operating normally
    DEGRADED,   // Sensor showing anomalies but still usable
    FAILED      // Sensor has failed, should not be used
};

/**
 * @brief Convert health status to string for logging
 */
inline const char* healthStatusToString(HealthStatus status) {
    switch (status) {
        case HealthStatus::HEALTHY:   return "HEALTHY";
        case HealthStatus::DEGRADED:  return "DEGRADED";
        case HealthStatus::FAILED:    return "FAILED";
        default:                      return "UNKNOWN";
    }
}

} // namespace fdir
} // namespace fsw
