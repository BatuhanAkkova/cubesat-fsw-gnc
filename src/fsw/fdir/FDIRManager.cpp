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

    // Default to 4 reaction wheels
    wheel_monitors_.resize(4);
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

void FDIRManager::updateWheels(const std::vector<double>& speeds, const std::vector<double>& commanded_torques,
                               double dt, double current_time) {
    if (wheel_monitors_.size() < speeds.size()) {
        wheel_monitors_.resize(speeds.size());
    }

    int failed_count_before = getFailedWheelCount();

    for (size_t i = 0; i < speeds.size(); ++i) {
        auto& wm = wheel_monitors_[i];

        // Estimate acceleration
        double delta_omega = speeds[i] - wm.last_speed;
        double accel = delta_omega / dt;
        wm.last_speed = speeds[i];

        // 1. Saturation detection
        // In our redundant system, max speed is around 100.0 rad/s
        double max_speed_threshold = 95.0;
        if (std::abs(speeds[i]) >= max_speed_threshold) {
            if (wm.status != HealthStatus::FAILED) {
                wm.status = HealthStatus::FAILED;
                common::LogWarning("[FDIR] Wheel {} detected as SATURATED (speed={:.2f} rad/s)", i, speeds[i]);
            }
            continue;
        }

        // 2. Stuck detection
        // If we command torque but wheel speed does not change
        double J_wheel = 0.001;          // assumed wheel inertia
        double friction_coeff = 0.0001;  // assumed wheel friction coefficient
        double expected_torque = commanded_torques[i] - friction_coeff * speeds[i];
        if (std::abs(expected_torque) > 2e-3) {
            double expected_accel = expected_torque / J_wheel;
            // If actual accel is near zero despite commanded torque
            if (std::abs(accel) < 0.05 * std::abs(expected_accel)) {
                wm.stuck_count++;
                if (wm.stuck_count >= 10) {  // 1.0 seconds at 10Hz
                    if (wm.status != HealthStatus::FAILED) {
                        wm.status = HealthStatus::FAILED;
                        common::LogError("[FDIR] Wheel {} detected as STUCK (speed={:.2f} rad/s, command={:.3e} Nm)", i,
                                         speeds[i], commanded_torques[i]);
                    }
                }
            } else {
                wm.stuck_count = 0;
            }

            // 3. Degraded torque detection
            // If actual accel is significantly less than expected (efficiency < 0.5)
            // but not completely stuck
            if (std::abs(accel) > 0.05 * std::abs(expected_accel) && std::abs(accel) < 0.5 * std::abs(expected_accel)) {
                wm.degraded_count++;
                if (wm.degraded_count >= 15) {
                    if (wm.status == HealthStatus::HEALTHY) {
                        wm.status = HealthStatus::DEGRADED;
                        common::LogWarning("[FDIR] Wheel {} detected as DEGRADED torque (accel ratio = {:.2f})", i,
                                           std::abs(accel / expected_accel));
                    }
                }
            } else {
                wm.degraded_count = 0;
            }
        } else {
            wm.stuck_count = 0;
            wm.degraded_count = 0;
        }
    }

    int failed_count_after = getFailedWheelCount();

    // Trigger state machine mode changes if failure count increased
    if (failed_count_after != failed_count_before) {
        if (auto_mode_transition_ && mode_manager_) {
            if (failed_count_after == 1) {
                common::LogWarning("[FDIR] Transitioning to DEGRADED mode due to 1 reaction wheel failure.");
                mode_manager_->forceModeChange(core::MissionMode::DEGRADED, "FDIR: 1 reaction wheel failure detected");
            } else if (failed_count_after >= 2) {
                common::LogError("[FDIR] Forcing transition to SAFE mode due to {} reaction wheel failures.",
                                 failed_count_after);
                mode_manager_->forceModeChange(core::MissionMode::SAFE,
                                               "FDIR: Multiple reaction wheel failures detected");
            }
        }
    }
}

HealthStatus FDIRManager::getWheelStatus(size_t index) const {
    if (index < wheel_monitors_.size()) {
        return wheel_monitors_[index].status;
    }
    return HealthStatus::FAILED;
}

int FDIRManager::getFailedWheelCount() const {
    int count = 0;
    for (const auto& wm : wheel_monitors_) {
        if (wm.status == HealthStatus::FAILED) {
            count++;
        }
    }
    return count;
}

}  // namespace fdir
}  // namespace fsw
