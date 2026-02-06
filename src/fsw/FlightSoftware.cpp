#include "fsw/FlightSoftware.hpp"
#include "fsw/core/CommandParser.hpp"
#include "fsw/core/commands/SetPidGainsCommand.hpp"
#include <iostream>

namespace fsw {

FlightSoftware::FlightSoftware(const Config& config) 
    : config_(config) {
    mode_manager_ = std::unique_ptr<core::ModeManager>(new core::ModeManager(config.mode_cfg));
    attitude_controller_ = std::unique_ptr<gnc::control::AttitudeController>(new gnc::control::AttitudeController(config.att_cfg));
    bdot_controller_ = std::unique_ptr<gnc::control::Bdot>(new gnc::control::Bdot(config.bdot_gain));
    telemetry_manager_ = std::unique_ptr<telemetry::TelemetryManager>(new telemetry::TelemetryManager(DataStore::Instance()));
    command_manager_ = std::unique_ptr<core::CommandManager>(new core::CommandManager());

    // Subscribe to guidance target changes from commands
    DataStore::Instance().subscribe<std::string>("guidance/target_mode", 
        [this](const std::string& mode) {
            this->guidance_mode_ = mode;
        });

    DataStore::Instance().subscribe<common::Quaternion>("guidance/target_quaternion",
        [this](const common::Quaternion& q) {
            this->target_q_ = q;
        });

    DataStore::Instance().subscribe<core::commands::GainsPayload>("gnc/att_gains",
        [this](const core::commands::GainsPayload& gains) {
            this->attitude_controller_->setGains(gains.kp, gains.ki, gains.kd, gains.is_nominal);
        });
}

common::Vector3 FlightSoftware::step(const SensorData& sensors, 
                                     const std::vector<std::vector<uint8_t>>& raw_commands,
                                     double dt) {
    if (dt <= 0) return common::Vector3::Zero(); 

    // Handle incoming commands
    for (const auto& raw_cmd : raw_commands) {
        auto cmd = core::CommandParser::parse(raw_cmd);
        if (cmd) {
            command_manager_->enqueueCommand(std::move(cmd));
        }
    }

    // Process commands
    command_manager_->update();

    // Update Mode Manager
    mode_manager_->update(sensors.gyro_body, dt);
    core::MissionMode mode = mode_manager_->getCurrentMode();

    common::Vector3 torque_cmd = common::Vector3::Zero();

    if (mode == core::MissionMode::SAFE) {
        // B-Dot Control
        common::Vector3 dipole = bdot_controller_->update(sensors.mag_body, dt);
        
        // Torque = dipole cross B
        torque_cmd = dipole.cross(sensors.mag_body);
        
    } else if (mode == core::MissionMode::NOMINAL) {
        // Dynamic Guidance based on guidance_mode_
        common::Quaternion q_target;
        
        if (guidance_mode_ == "NADIR") {
            // Need position/velocity for Nadir, but SensorData is simplified.
            // For now, assume a mock Nadir vector or get it from DataStore.
            common::Vector3 sc_pos(1e6, 0, 0); // Mock
            common::Vector3 sc_vel(0, 7500, 0); // Mock
            q_target = gnc::guidance::PointingStrategies::nadirPointing(sc_pos, sc_vel);
        } else if (guidance_mode_ == "TARGET") {
            q_target = target_q_;
        } else {
            // Default: Point +Z at Sun
            common::Vector3 body_axis(0, 0, 1);
            q_target = gnc::guidance::PointingStrategies::alignAxis(
                body_axis, config_.sun_inertial);
        }
        
        torque_cmd = attitude_controller_->computeTorque(
            sensors.q_measured, q_target, sensors.gyro_body, dt);
    }

    // Update Telemetry
    telemetry_manager_->update(dt);

    last_torque_cmd_ = torque_cmd;
    return torque_cmd;
}

} // namespace fsw
