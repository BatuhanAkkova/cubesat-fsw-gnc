#pragma once

#include "common/types.hpp"
#include "fsw/core/ModeManager.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/control/Bdot.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"
#include "fsw/telemetry/TelemetryManager.hpp"
#include "fsw/core/CommandManager.hpp"
#include <memory>
#include <vector>
#include <string>

namespace fsw {

/**
 * @brief Struct to hold all sensor measurements for a mission step.
 */
struct SensorData {
    common::Vector3 mag_body;       // Tesla
    common::Vector3 gyro_body;      // rad/s
    common::Quaternion q_measured;  // Attitude estimate (from Star Tracker or MEKF)
    common::Vector3 sun_body;       // Sun vector in body frame
};

/**
 * @brief FlightSoftware wrapper to simplify execution in simulations.
 * This class orchestrates the ModeManager and various controllers.
 */
class FlightSoftware {
public:
    struct Config {
        core::ModeTransitionConfig mode_cfg;
        gnc::control::AttitudeController::Config att_cfg;
        double bdot_gain = 50000.0;
        common::Vector3 sun_inertial = common::Vector3(1, 0, 0); 
    };

    FlightSoftware(const Config& config);

    /**
     * @brief Run one iteration of the FSW.
     * @param sensors Latest sensor measurements.
     * @param raw_commands Raw CCSDS command packets received since last step.
     * @param dt Time step [seconds]
     * @return Commanded torque in body frame [Nm]
     */
    common::Vector3 step(const SensorData& sensors, 
                         const std::vector<std::vector<uint8_t>>& raw_commands,
                         double dt);

    // Accessors for optimization/testing
    gnc::control::AttitudeController& getAttitudeController() { return *attitude_controller_; }
    core::ModeManager& getModeManager() { return *mode_manager_; }
    
    void setTargetSunInertial(const common::Vector3& sun) { config_.sun_inertial = sun; }
    void setConfigInertia(const common::Matrix3& inertia) { /* Placeholder if needed for LQG */ }

    common::Vector3 getCommandTorque() const { return last_torque_cmd_; }
    core::MissionMode getCurrentMode() const { return mode_manager_->getCurrentMode(); }

private:
    Config config_;
    std::unique_ptr<core::ModeManager> mode_manager_;
    std::unique_ptr<gnc::control::AttitudeController> attitude_controller_;
    std::unique_ptr<gnc::control::Bdot> bdot_controller_;
    std::unique_ptr<telemetry::TelemetryManager> telemetry_manager_;
    std::unique_ptr<core::CommandManager> command_manager_;
    
    std::string guidance_mode_ = "SUN"; // Default pointing mode
    common::Quaternion target_q_ = common::Quaternion::Identity();
    common::Vector3 last_torque_cmd_ = common::Vector3::Zero();
};

} // namespace fsw
