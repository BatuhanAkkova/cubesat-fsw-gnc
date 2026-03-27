#include "FDIRManager.hpp"

#include <unordered_set>

#include "fsw/core/ModeManager.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"

namespace fsw {
namespace fdir {

FDIRManager::FDIRManager() {
    // Mark gyroscope as critical sensor
    critical_sensors_.insert("gyro");
    critical_sensors_.insert("gyro_primary");
    critical_sensors_.insert("gyro_backup");
}

void FDIRManager::registerRedundantPair(const std::string& group_name, const std::string& primary_name,
                                        const std::string& backup_name) {
    RedundancyGroup group;
    group.primary = primary_name;
    group.backup = backup_name;
    group.active = primary_name;
    group.backup_available = true;

    redundancy_groups_[group_name] = group;
    sensor_to_group_[primary_name] = group_name;
    sensor_to_group_[backup_name] = group_name;

    common::LogInfo("[FDIR] Registered redundancy group: {} (P: {}, B: {})", group_name, primary_name, backup_name);
}

std::string FDIRManager::getActiveSensor(const std::string& group_name) const {
    auto it = redundancy_groups_.find(group_name);
    if (it != redundancy_groups_.end()) {
        return it->second.active;
    }
    return "";
}

SensorHealthMonitor<common::Vector3>* FDIRManager::registerVector3Sensor(
    const std::string& sensor_name, std::function<void(SensorHealthMonitor<common::Vector3>&)> config_setter) {
    auto monitor = std::make_unique<SensorHealthMonitor<common::Vector3>>(sensor_name);

    // Apply configuration if provided
    if (config_setter) {
        config_setter(*monitor);
    }

    // Set up status change callback
    monitor->setStatusChangeCallback([this](const std::string& name, HealthStatus status, const std::string& msg) {
        this->handleStatusChange(name, status, msg);
    });

    auto* ptr = monitor.get();
    vector_monitors_[sensor_name] = std::move(monitor);

    common::LogInfo("[FDIR] Registered sensor: {}", sensor_name);
    return ptr;
}

SensorHealthMonitor<common::Vector3>* FDIRManager::registerGyro(const std::string& sensor_name,
                                                                const GyroHealthConfig& config) {
    return registerVector3Sensor(sensor_name, [&config](SensorHealthMonitor<common::Vector3>& monitor) {
        monitor.setStuckThreshold(config.stuck_tolerance, config.stuck_sample_count);
        monitor.setRangeThresholds(0.0, config.max_rate);
        monitor.setRapidChangeThreshold(config.max_rate_change, config.dt);
    });
}

SensorHealthMonitor<common::Vector3>* FDIRManager::registerMagnetometer(const std::string& sensor_name,
                                                                        const MagnetometerHealthConfig& config) {
    return registerVector3Sensor(sensor_name, [&config](SensorHealthMonitor<common::Vector3>& monitor) {
        monitor.setStuckThreshold(config.stuck_tolerance, config.stuck_sample_count);
        monitor.setRangeThresholds(config.min_field, config.max_field);
        monitor.setRapidChangeThreshold(config.max_field_change, config.dt);
    });
}

void FDIRManager::updateSensor(const std::string& sensor_name, const common::Vector3& measurement,
                               double current_time) {
    auto it = vector_monitors_.find(sensor_name);
    if (it != vector_monitors_.end()) {
        it->second->update(measurement, current_time);
    } else {
        common::LogWarning("[FDIR] Unknown sensor: {}", sensor_name);
    }
}

void FDIRManager::updateSensor(const std::string& sensor_name, const common::Quaternion& measurement,
                               double current_time) {
    // For quaternion sensors (star tracker), we'll convert to angle-axis for monitoring
    // This monitors the attitude angle magnitude
    common::AngleAxis aa(measurement);
    common::Vector3 axis_angle = aa.axis() * aa.angle();

    updateSensor(sensor_name, axis_angle, current_time);
}

HealthStatus FDIRManager::getSensorStatus(const std::string& sensor_name) const {
    auto it = vector_monitors_.find(sensor_name);
    if (it != vector_monitors_.end()) {
        return it->second->getStatus();
    }
    return HealthStatus::FAILED;  // Unknown sensor treated as failed
}

HealthStatus FDIRManager::getSystemHealth() const {
    HealthStatus worst = HealthStatus::HEALTHY;

    for (const auto& [name, monitor] : vector_monitors_) {
        HealthStatus status = monitor->getStatus();
        if (status == HealthStatus::FAILED) {
            return HealthStatus::FAILED;  // Any failure means system is failed
        }
        if (status == HealthStatus::DEGRADED) {
            worst = HealthStatus::DEGRADED;
        }
    }

    return worst;
}

FDIRManager::HealthSummary FDIRManager::getHealthSummary() const {
    HealthSummary summary;

    for (const auto& [name, monitor] : vector_monitors_) {
        switch (monitor->getStatus()) {
            case HealthStatus::HEALTHY:
                summary.healthy_count++;
                break;
            case HealthStatus::DEGRADED:
                summary.degraded_count++;
                break;
            case HealthStatus::FAILED:
                summary.failed_count++;
                break;
        }
    }

    return summary;
}

void FDIRManager::resetAll() {
    common::LogInfo("[FDIR] Resetting all sensor monitors");

    for (auto& [name, monitor] : vector_monitors_) {
        monitor->reset();
    }
}

void FDIRManager::handleStatusChange(const std::string& sensor_name, HealthStatus new_status,
                                     const std::string& message) {
    // Log the status change
    common::LogWarning("[FDIR] {}: {} - {}", sensor_name, healthStatusToString(new_status), message);

    // Execute response actions
    executeResponseActions(sensor_name, new_status);
}

void FDIRManager::executeResponseActions(const std::string& sensor_name, HealthStatus status) {
    // Check if this sensor belongs to a redundancy group
    auto group_it = sensor_to_group_.find(sensor_name);
    bool has_redundancy = (group_it != sensor_to_group_.end());

    if (status == HealthStatus::FAILED) {
        common::LogError("[FDIR] Sensor FAILED: {}", sensor_name);

        bool failover_success = false;

        if (has_redundancy) {
            std::string group_name = group_it->second;
            auto& group = redundancy_groups_[group_name];

            // If primary failed and we are on primary, switch to backup
            if (sensor_name == group.primary && group.active == group.primary && group.backup_available) {
                common::LogWarning("[FDIR] Primary sensor {} FAILED. Switching to backup: {}", sensor_name,
                                   group.backup);
                group.active = group.backup;
                group.backup_available = false;  // Backup is now primary, no more backups
                failover_success = true;
            }
            // If we are already on backup and it fails, or it's the backup that failed while we're on primary
            else if (sensor_name == group.backup) {
                group.backup_available = false;
                if (group.active == group.backup) {
                    common::LogError("[FDIR] Backup sensor {} FAILED. No more sensors in group {}", sensor_name,
                                     group_name);
                }
            }
        }

        // If failover happened, we might NOT need to go to SAFE mode immediately if the backup is healthy
        // But for this implementation, we still consider the system "degraded" after a failover.
        if (failover_success) {
            if (auto_mode_transition_ && mode_manager_) {
                common::LogWarning("[FDIR] Transitioning to DEGRADED mode after sensor failover");
                mode_manager_->forceModeChange(core::MissionMode::DEGRADED,
                                               "FDIR: Primary sensor failure, switched to backup - " + sensor_name);
            }
            return;  // Skip the SAFE mode transition below
        }

        // Check if this is a critical sensor failure (and no successful failover)
        bool is_critical = isCriticalSensor(sensor_name);

        // Reset MEKF if enabled and this is a critical sensor
        if (auto_mekf_reset_ && is_critical && mekf_) {
            common::LogWarning("[FDIR] Resetting MEKF due to critical sensor failure");
            mekf_->reset();
        }

        // Transition to SAFE mode if enabled and this is a critical sensor
        if (auto_mode_transition_ && is_critical && mode_manager_) {
            common::LogWarning("[FDIR] Forcing transition to SAFE mode due to critical sensor failure: {}",
                               sensor_name);
            mode_manager_->forceModeChange(core::MissionMode::SAFE, "FDIR: Critical sensor failure - " + sensor_name);
        }
    } else if (status == HealthStatus::DEGRADED) {
        common::LogWarning("[FDIR] Sensor DEGRADED: {} - continuing with reduced confidence", sensor_name);

        // Transition to DEGRADED mode if not already in a worse mode
        if (auto_mode_transition_ && mode_manager_ && mode_manager_->getCurrentMode() == core::MissionMode::NOMINAL) {
            common::LogWarning("[FDIR] Transitioning to DEGRADED mode due to sensor degradation: {}", sensor_name);
            mode_manager_->forceModeChange(core::MissionMode::DEGRADED, "FDIR: Sensor degraded - " + sensor_name);
        }
    }
}

bool FDIRManager::isCriticalSensor(const std::string& sensor_name) const {
    return critical_sensors_.find(sensor_name) != critical_sensors_.end();
}

}  // namespace fdir
}  // namespace fsw
