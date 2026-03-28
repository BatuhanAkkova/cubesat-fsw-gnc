#pragma once

#include <deque>
#include <functional>
#include <string>

#include "common/logger.hpp"
#include "common/types.hpp"

#include "FDIRConfig.hpp"

namespace fsw {
namespace fdir {

/**
 * @brief Generic sensor health monitor with multiple detection strategies
 *
 * This class monitors a scalar or vector sensor for common failure modes:
 * - Stuck sensor (no change for N consecutive samples)
 * - Out-of-range values
 * - Rapid changes (unrealistic jumps)
 *
 * Template parameter T should be common::Vector3 or double
 */
template <typename T>
class SensorHealthMonitor {
   public:
    /**
     * @brief Response action callback type
     * @param sensor_name Name of the sensor that triggered the response
     * @param status Current health status
     * @param message Descriptive message about the fault
     */
    using ResponseCallback = std::function<void(const std::string&, HealthStatus, const std::string&)>;

    /**
     * @brief Constructor
     * @param sensor_name Identifier for this sensor (for logging)
     */
    explicit SensorHealthMonitor(const std::string& sensor_name)
        : sensor_name_(sensor_name),
          current_status_(HealthStatus::HEALTHY),
          stuck_count_(0),
          enable_stuck_detection_(true),
          enable_range_detection_(true),
          enable_rapid_change_detection_(true) {}

    /**
     * @brief Update health monitor with new sensor reading
     * @param measurement New sensor measurement
     * @param current_time Current simulation/mission time [seconds]
     */
    void update(const T& measurement, double current_time) {
        // Add to history
        addToHistory(measurement, current_time);

        // Run detection strategies
        HealthStatus new_status = HealthStatus::HEALTHY;
        std::string fault_message;

        if (enable_stuck_detection_ && detectStuck(measurement, fault_message)) {
            new_status = HealthStatus::FAILED;
        } else if (enable_range_detection_ && detectOutOfRange(measurement, fault_message)) {
            new_status = HealthStatus::DEGRADED;
        } else if (enable_rapid_change_detection_ && detectRapidChange(measurement, fault_message)) {
            new_status = HealthStatus::DEGRADED;
        }
        // Note: stuck_count_ is managed within detectStuck() itself

        // Check if status changed
        if (new_status != current_status_) {
            common::LogWarning("[FDIR] {} status changed: {} -> {}", sensor_name_,
                               healthStatusToString(current_status_), healthStatusToString(new_status));

            current_status_ = new_status;

            // Trigger callbacks
            if (on_status_change_) {
                on_status_change_(sensor_name_, current_status_, fault_message);
            }
        }
    }

    /**
     * @brief Get current health status
     */
    HealthStatus getStatus() const {
        return current_status_;
    }

    /**
     * @brief Get sensor name
     */
    const std::string& getName() const {
        return sensor_name_;
    }

    /**
     * @brief Register callback for status changes
     */
    void setStatusChangeCallback(ResponseCallback callback) {
        on_status_change_ = callback;
    }

    /**
     * @brief Enable/disable specific detection strategies
     */
    void enableStuckDetection(bool enable) {
        enable_stuck_detection_ = enable;
    }
    void enableRangeDetection(bool enable) {
        enable_range_detection_ = enable;
    }
    void enableRapidChangeDetection(bool enable) {
        enable_rapid_change_detection_ = enable;
    }

    /**
     * @brief Reset the monitor to healthy state
     */
    void reset() {
        current_status_ = HealthStatus::HEALTHY;
        stuck_count_ = 0;
        measurement_history_.clear();
        time_history_.clear();
    }

    /**
     * @brief Set configuration parameters (must be specialized for each sensor type)
     */
    virtual void setStuckThreshold(double tolerance, int sample_count) {
        stuck_tolerance_ = tolerance;
        stuck_sample_count_ = sample_count;
    }

    virtual void setRangeThresholds(double min_val, double max_val) {
        min_value_ = min_val;
        max_value_ = max_val;
    }

    virtual void setRapidChangeThreshold(double max_change, double dt) {
        max_change_per_sample_ = max_change * dt;
    }

   protected:
    /**
     * @brief Add measurement to history buffer
     */
    void addToHistory(const T& measurement, double time) {
        measurement_history_.push_back(measurement);
        time_history_.push_back(time);

        // Keep only recent history (avoid unbounded growth)
        constexpr size_t MAX_HISTORY = 100;
        if (measurement_history_.size() > MAX_HISTORY) {
            measurement_history_.pop_front();
            time_history_.pop_front();
        }
    }

    /**
     * @brief Compute magnitude/norm of measurement (specialized for Vector3 and double)
     */
    double computeMagnitude(const T& measurement) const;

    /**
     * @brief Compute difference between measurements (specialized for Vector3 and double)
     */
    double computeDifference(const T& a, const T& b) const;

    /**
     * @brief Detect stuck sensor
     */
    virtual bool detectStuck(const T& measurement, std::string& message) {
        if (measurement_history_.size() < 2) {
            return false;  // Not enough data yet
        }

        // Check if sensor hasn't changed significantly
        const T& prev = measurement_history_[measurement_history_.size() - 2];
        double diff = computeDifference(measurement, prev);

        if (diff < stuck_tolerance_) {
            stuck_count_++;
            if (stuck_count_ >= stuck_sample_count_) {
                message = "Sensor stuck - no change for " + std::to_string(stuck_count_) + " samples";
                return true;
            }
        } else {
            stuck_count_ = 0;
        }

        return false;
    }

    /**
     * @brief Detect out-of-range values
     */
    virtual bool detectOutOfRange(const T& measurement, std::string& message) {
        double mag = computeMagnitude(measurement);

        if (mag < min_value_ || mag > max_value_) {
            message = "Out of range: " + std::to_string(mag) + " (expected [" + std::to_string(min_value_) + ", " +
                      std::to_string(max_value_) + "])";
            return true;
        }

        return false;
    }

    /**
     * @brief Detect rapid changes
     */
    virtual bool detectRapidChange(const T& measurement, std::string& message) {
        if (measurement_history_.size() < 2) {
            return false;
        }

        const T& prev = measurement_history_[measurement_history_.size() - 2];
        double change = computeDifference(measurement, prev);

        if (change > max_change_per_sample_) {
            message = "Rapid change detected: " + std::to_string(change) +
                      " (max: " + std::to_string(max_change_per_sample_) + ")";
            return true;
        }

        return false;
    }

    // Member variables
    std::string sensor_name_;
    HealthStatus current_status_;

    // History buffers
    std::deque<T> measurement_history_;
    std::deque<double> time_history_;

    // Detection parameters
    double stuck_tolerance_ = 1e-6;
    int stuck_sample_count_ = 50;
    int stuck_count_;

    double min_value_ = 0.0;
    double max_value_ = 1e9;

    double max_change_per_sample_ = 1e9;

    // Feature flags
    bool enable_stuck_detection_;
    bool enable_range_detection_;
    bool enable_rapid_change_detection_;

    // Callbacks
    ResponseCallback on_status_change_;
};

// Template specializations for magnitude computation
template <>
inline double SensorHealthMonitor<common::Vector3>::computeMagnitude(const common::Vector3& measurement) const {
    return measurement.norm();
}

template <>
inline double SensorHealthMonitor<double>::computeMagnitude(const double& measurement) const {
    return std::abs(measurement);
}

// Template specializations for difference computation
template <>
inline double SensorHealthMonitor<common::Vector3>::computeDifference(const common::Vector3& a,
                                                                      const common::Vector3& b) const {
    return (a - b).norm();
}

template <>
inline double SensorHealthMonitor<double>::computeDifference(const double& a, const double& b) const {
    return std::abs(a - b);
}

}  // namespace fdir
}  // namespace fsw
