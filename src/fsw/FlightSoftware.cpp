#include "fsw/FlightSoftware.hpp"

#include <iostream>

#include "fsw/core/CommandParser.hpp"
#include "fsw/core/DataStore.hpp"
#include "fsw/core/commands/SetPidGainsCommand.hpp"
#include "fsw/gnc/GNCComponentFactory.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"

namespace fsw {

FlightSoftware::FlightSoftware(const Config& config) : config_(config) {
    mode_manager_ = std::make_unique<core::ModeManager>(config.mode_cfg);

    // Initialize GNC components via Factory
    if (config.full_config.contains("fsw")) {
        auto fsw_json = config.full_config["fsw"];

        if (fsw_json.contains("estimator")) {
            estimator_ = gnc::GNCComponentFactory::createEstimator(fsw_json["estimator"]);
        }

        if (fsw_json.contains("controllers")) {
            auto ctrl_json = fsw_json["controllers"];
            if (ctrl_json.contains("attitude")) {
                attitude_controller_ = gnc::GNCComponentFactory::createController(ctrl_json["attitude"]);
            }
            if (ctrl_json.contains("detumble")) {
                bdot_controller_ = gnc::GNCComponentFactory::createController(ctrl_json["detumble"]);
            }
        }
    }

    telemetry_manager_ = std::make_unique<telemetry::TelemetryManager>(DataStore::Instance());
    command_manager_ = std::make_unique<core::CommandManager>();

    // Subscribe to topics
    DataStore::Instance().subscribe<std::string>("guidance/target_mode",
                                                 [this](const std::string& mode) { this->guidance_mode_ = mode; });

    DataStore::Instance().subscribe<common::Quaternion>("guidance/target_quaternion",
                                                        [this](const common::Quaternion& q) { this->target_q_ = q; });

    // Initialize lock-free queue and state history
    telemetry_queue_ = std::make_unique<core::SPSCQueue<common::State, 128>>();
    state_history_ = std::make_unique<common::StateHistory>(1000);  // 1000 samples
}

common::Vector3 FlightSoftware::step(const common::SensorData& sensors,
                                     const std::vector<std::vector<uint8_t>>& raw_commands, double dt) {
    if (dt <= 0) return common::Vector3::Zero();

    // 1. Process Commands
    for (const auto& raw_cmd : raw_commands) {
        auto cmd = core::CommandParser::parse(raw_cmd);
        if (cmd) command_manager_->enqueueCommand(std::move(cmd));
    }
    command_manager_->update();

    // 2. State Estimation
    common::State state_est;
    if (estimator_) {
        estimator_->update(sensors, dt);
        state_est.q = estimator_->getAttitude();
        state_est.w = estimator_->getAngularVelocity();
    } else {
        // Fallback to "measured" data if no estimator
        state_est.q = sensors.q_measured;
        state_est.w = sensors.gyro_body;
    }

    // 3. Mode Management
    mode_manager_->update(state_est.w, dt);
    core::MissionMode mode = mode_manager_->getCurrentMode();

    // 4. Guidance & Control
    common::Vector3 torque_cmd = common::Vector3::Zero();
    common::GuidanceTarget target;
    target.mode = guidance_mode_;

    if (mode == core::MissionMode::SAFE) {
        if (bdot_controller_) {
            torque_cmd = bdot_controller_->update(sensors, state_est, target, dt);
            // In B-Dot mode, torque = dipole cross B if Bdot returns dipole,
            // but our Bdot implementation returns Torque for simplicity in this sim.
            // If Bdot returns dipole: torque_cmd = torque_cmd.cross(sensors.mag_body);
        }
    } else if (mode == core::MissionMode::NOMINAL) {
        // Guidance
        if (guidance_mode_ == "NADIR") {
            common::Vector3 sc_pos(1e6, 0, 0);   // Mock
            common::Vector3 sc_vel(0, 7500, 0);  // Mock
            target.q = gnc::guidance::PointingStrategies::nadirPointing(sc_pos, sc_vel);
        } else if (guidance_mode_ == "TARGET") {
            target.q = target_q_;
        } else {
            target.q = gnc::guidance::PointingStrategies::alignAxis(common::Vector3(0, 0, 1), config_.sun_inertial);
        }
        target.w = common::Vector3::Zero();  // Assume static target rates for now

        if (attitude_controller_) {
            torque_cmd = attitude_controller_->update(sensors, state_est, target, dt);
        }
    }

    // Update lock-free telemetry and SIMD state history
    telemetry_queue_->push(state_est);
    state_history_->addState(state_est);

    // 5. Telemetry & Cleanup
    telemetry_manager_->update(dt);
    last_torque_cmd_ = torque_cmd;
    return torque_cmd;
}

}  // namespace fsw
