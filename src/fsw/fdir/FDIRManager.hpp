#pragma once

#include "SensorHealthMonitor.hpp"
#include "FDIRConfig.hpp"
#include "common/types.hpp"
#include "common/logger.hpp"
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>

// Forward declarations
namespace fsw {
namespace core {
    class ModeManager;
}
namespace gnc {
namespace ekf {
    class MEKF;
}
}
}

namespace fsw {
namespace fdir {

/**
 * @brief System-wide FDIR manager coordinating all sensor health monitors
 * 
 * Responsibilities:
 * - Manage multiple SensorHealthMonitor instances
 * - Aggregate system health status
 * - Trigger system-level responses (mode transitions, MEKF resets)
 * - Publish health telemetry
 */
class FDIRManager {
public:
    /**
     * @brief Constructor
     */
    FDIRManager();

    /**
     * @brief Register a Vector3 sensor monitor (gyro, magnetometer)
     * @param sensor_name Unique identifier for the sensor
     * @param config_setter Function to configure the monitor
     * @return Pointer to the registered monitor
     */
    SensorHealthMonitor<common::Vector3>* registerVector3Sensor(
        const std::string& sensor_name,
        std::function<void(SensorHealthMonitor<common::Vector3>&)> config_setter = nullptr);

    /**
     * @brief Register gyroscope with standard configuration
     * @param sensor_name Unique identifier
     * @param config Gyro-specific configuration
     */
    SensorHealthMonitor<common::Vector3>* registerGyro(
        const std::string& sensor_name,
        const GyroHealthConfig& config = GyroHealthConfig());

    /**
     * @brief Register magnetometer with standard configuration
     * @param sensor_name Unique identifier
     * @param config Magnetometer-specific configuration
     */
    SensorHealthMonitor<common::Vector3>* registerMagnetometer(
        const std::string& sensor_name,
        const MagnetometerHealthConfig& config = MagnetometerHealthConfig());

    /**
     * @brief Update a specific sensor monitor
     * @param sensor_name Name of the sensor
     * @param measurement Sensor reading
     * @param current_time Mission time [seconds]
     */
    void updateSensor(const std::string& sensor_name, 
                      const common::Vector3& measurement,
                      double current_time);

    /**
     * @brief Update a specific sensor monitor (quaternion version for star tracker)
     * @param sensor_name Name of the sensor
     * @param measurement Sensor reading
     * @param current_time Mission time [seconds]
     */
    void updateSensor(const std::string& sensor_name,
                      const common::Quaternion& measurement,
                      double current_time);

    /**
     * @brief Get health status of a specific sensor
     * @param sensor_name Name of the sensor
     * @return Health status, or FAILED if sensor not found
     */
    HealthStatus getSensorStatus(const std::string& sensor_name) const;

    /**
     * @brief Get overall system health status
     * @return Worst status among all sensors
     */
    HealthStatus getSystemHealth() const;

    /**
     * @brief Set mode manager for automatic mode transitions
     * @param mode_manager Pointer to the mode manager
     */
    void setModeManager(core::ModeManager* mode_manager) {
        mode_manager_ = mode_manager;
    }

    /**
     * @brief Set MEKF for reset capability
     * @param mekf Pointer to the MEKF instance
     */
    void setMEKF(gnc::ekf::MEKF* mekf) {
        mekf_ = mekf;
    }

    /**
     * @brief Reset all monitors to healthy state
     */
    void resetAll();

    /**
     * @brief Get count of sensors by status
     */
    struct HealthSummary {
        int healthy_count = 0;
        int degraded_count = 0;
        int failed_count = 0;
    };
    HealthSummary getHealthSummary() const;

    /**
     * @brief Enable/disable automatic MEKF reset on sensor failure
     */
    void enableAutoMEKFReset(bool enable) { auto_mekf_reset_ = enable; }

    /**
     * @brief Enable/disable automatic mode transitions on critical failure
     */
    void enableAutoModeTransition(bool enable) { auto_mode_transition_ = enable; }

    /**
     * @brief Register a redundant sensor pair
     * @param group_name Name of the logical sensor group (e.g., "gyro")
     * @param primary_name Name of the primary sensor
     * @param backup_name Name of the backup sensor
     */
    void registerRedundantPair(const std::string& group_name,
                               const std::string& primary_name,
                               const std::string& backup_name);

    /**
     * @brief Get the currently active sensor for a group
     * @param group_name Name of the sensor group
     * @return Name of the active sensor, or empty if group not found
     */
    std::string getActiveSensor(const std::string& group_name) const;

private:
    /**
     * @brief Handle sensor status change events
     */
    void handleStatusChange(const std::string& sensor_name, 
                           HealthStatus new_status,
                           const std::string& message);

    /**
     * @brief Execute response actions based on sensor failure
     */
    void executeResponseActions(const std::string& sensor_name, HealthStatus status);

    /**
     * @brief Check if a sensor is critical (requires immediate action)
     */
    bool isCriticalSensor(const std::string& sensor_name) const;

    // Sensor monitors storage
    std::unordered_map<std::string, std::unique_ptr<SensorHealthMonitor<common::Vector3>>> vector_monitors_;
    
    // Redundancy management
    struct RedundancyGroup {
        std::string primary;
        std::string backup;
        std::string active;
        bool backup_available = true;
    };
    std::unordered_map<std::string, RedundancyGroup> redundancy_groups_;
    std::unordered_map<std::string, std::string> sensor_to_group_;

    // External component references
    core::ModeManager* mode_manager_ = nullptr;
    gnc::ekf::MEKF* mekf_ = nullptr;

    // Configuration flags
    bool auto_mekf_reset_ = true;
    bool auto_mode_transition_ = true;

    // Critical sensor list (sensors whose failure requires immediate action)
    std::unordered_set<std::string> critical_sensors_;
};

} // namespace fdir
} // namespace fsw
