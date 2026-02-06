#include "fsw/FlightSoftware.hpp"
#include <iostream>

namespace fsw {

FlightSoftware::FlightSoftware(const Config& config) 
    : config_(config) {
    mode_manager_ = std::unique_ptr<core::ModeManager>(new core::ModeManager(config.mode_cfg));
    attitude_controller_ = std::unique_ptr<gnc::control::AttitudeController>(new gnc::control::AttitudeController(config.att_cfg));
    bdot_controller_ = std::unique_ptr<gnc::control::Bdot>(new gnc::control::Bdot(config.bdot_gain));
}

common::Vector3 FlightSoftware::step(const SensorData& sensors, double dt) {
    if (dt <= 0) return common::Vector3::Zero(); 
    // Update Mode Manager
    mode_manager_->update(sensors.gyro_body, dt);
    core::MissionMode mode = mode_manager_->getCurrentMode();

    common::Vector3 torque_cmd = common::Vector3::Zero();

    if (mode == core::MissionMode::SAFE) {
        // B-Dot Control: dipole = -k * dB/dt
        // In this implementation, Bdot handles the dipole calculation
        common::Vector3 dipole = bdot_controller_->update(sensors.mag_body, dt);
        
        // Torque = dipole cross B
        torque_cmd = dipole.cross(sensors.mag_body);
        
    } else if (mode == core::MissionMode::NOMINAL) {
        // Attitude Control: Point +Z at Sun
        common::Vector3 body_axis(0, 0, 1);
        common::Quaternion q_target = gnc::guidance::PointingStrategies::alignAxis(
            body_axis, config_.sun_inertial);
        
        torque_cmd = attitude_controller_->computeTorque(
            sensors.q_measured, q_target, sensors.gyro_body, dt);
    }

    last_torque_cmd_ = torque_cmd;
    return torque_cmd;
}

} // namespace fsw
