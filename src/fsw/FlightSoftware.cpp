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
    fdir_manager_ = std::make_unique<fdir::FDIRManager>();
    fdir_manager_->setModeManager(mode_manager_.get());
    control_allocator_ = std::make_unique<gnc::control::ControlAllocator>();
    last_rw_torque_cmds_.assign(4, 0.0);

    // Initialize GNC components via Factory
    if (config.full_config.contains("fsw")) {
        auto fsw_json = config.full_config["fsw"];

        if (fsw_json.contains("estimator")) {
            estimator_ = gnc::GNCComponentFactory::createEstimator(fsw_json["estimator"]);

            // Link MEKF with FDIR manager if applicable
            auto* mekf_ptr = dynamic_cast<gnc::ekf::MEKF*>(estimator_.get());
            if (mekf_ptr) {
                fdir_manager_->setMEKF(mekf_ptr);
            }
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

    // Update accumulated time
    accumulated_time_ += dt;

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

    // 3. Actuator FDIR & Control Allocation Configuration
    if (sensors.rw_speeds.size() == 4) {
        // Run reaction wheel FDIR using current speeds and previously commanded torques
        fdir_manager_->updateWheels(sensors.rw_speeds, last_rw_torque_cmds_, dt, accumulated_time_);

        // Update allocator healthy flags from FDIR status
        for (size_t i = 0; i < 4; ++i) {
            bool healthy = (fdir_manager_->getWheelStatus(i) != fdir::HealthStatus::FAILED);
            control_allocator_->setWheelHealth(i, healthy);
        }
    }

    // 4. Mode Management
    mode_manager_->update(state_est.w, dt);
    core::MissionMode mode = mode_manager_->getCurrentMode();

    // 5. Guidance & Control
    common::Vector3 torque_cmd = common::Vector3::Zero();
    common::GuidanceTarget target;
    target.mode = guidance_mode_;

    if (mode == core::MissionMode::SAFE) {
        if (bdot_controller_) {
            torque_cmd = bdot_controller_->update(sensors, state_est, target, dt);
        }
        // In SAFE mode, command reaction wheels to 0
        last_rw_torque_cmds_.assign(4, 0.0);
    } else {
        // Pointing control (NOMINAL, DEGRADED, SCIENCE, DOWNLINK)
        if (guidance_mode_ == "NADIR") {
            common::Vector3 sc_pos(1e6, 0, 0);   // Mock
            common::Vector3 sc_vel(0, 7500, 0);  // Mock
            target.q = gnc::guidance::PointingStrategies::nadirPointing(sc_pos, sc_vel);
        } else if (guidance_mode_ == "TARGET") {
            target.q = target_q_;
        } else {
            target.q = gnc::guidance::PointingStrategies::alignAxis(common::Vector3(0, 0, 1), config_.sun_inertial);
        }
        target.w = common::Vector3::Zero();

        if (attitude_controller_) {
            torque_cmd = attitude_controller_->update(sensors, state_est, target, dt);
        }

        // Run control allocation if using redundant reaction wheels
        if (sensors.rw_speeds.size() == 4) {
            std::vector<double> allocated_torques;
            bool success = control_allocator_->allocate(torque_cmd, allocated_torques);
            if (success) {
                last_rw_torque_cmds_ = allocated_torques;
            } else {
                // If allocation fails (det < threshold because too many failed wheels), force SAFE mode
                common::LogError("[FSW] Allocation failed. Forcing transition to SAFE mode.");
                mode_manager_->forceModeChange(core::MissionMode::SAFE, "FDIR: Actuator allocation failure");
                last_rw_torque_cmds_.assign(4, 0.0);
                torque_cmd = common::Vector3::Zero();
            }
        } else {
            // Direct 3-axis fallback for backwards compatibility
            last_rw_torque_cmds_.clear();
        }
    }

    // Update lock-free telemetry and SIMD state history
    telemetry_queue_->push(state_est);
    state_history_->addState(state_est);

    // 6. Telemetry & Cleanup
    telemetry_manager_->update(dt);
    last_torque_cmd_ = torque_cmd;
    return torque_cmd;
}

}  // namespace fsw
