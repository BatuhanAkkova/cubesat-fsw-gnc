#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "fsw/core/CommandManager.hpp"
#include "fsw/core/ModeManager.hpp"
#include "fsw/gnc/interfaces/IController.hpp"
#include "fsw/gnc/interfaces/IEstimator.hpp"
#include "fsw/telemetry/TelemetryManager.hpp"

namespace fsw {

/**
 * @brief FlightSoftware wrapper to simplify execution in simulations.
 */
class FlightSoftware {
   public:
    struct Config {
        nlohmann::json full_config;  // Store the full JSON for factory use
        core::ModeTransitionConfig mode_cfg;
        common::Vector3 sun_inertial = common::Vector3(1, 0, 0);
    };

    FlightSoftware(const Config& config);

    common::Vector3 step(const common::SensorData& sensors, const std::vector<std::vector<uint8_t>>& raw_commands,
                         double dt);

    core::ModeManager& getModeManager() {
        return *mode_manager_;
    }
    common::Vector3 getCommandTorque() const {
        return last_torque_cmd_;
    }
    core::MissionMode getCurrentMode() const {
        return mode_manager_->getCurrentMode();
    }

   private:
    Config config_;
    std::unique_ptr<core::ModeManager> mode_manager_;
    std::unique_ptr<gnc::interfaces::IEstimator> estimator_;
    std::unique_ptr<gnc::interfaces::IController> attitude_controller_;
    std::unique_ptr<gnc::interfaces::IController> bdot_controller_;
    std::unique_ptr<telemetry::TelemetryManager> telemetry_manager_;
    std::unique_ptr<core::CommandManager> command_manager_;

    std::string guidance_mode_ = "SUN";
    common::Quaternion target_q_ = common::Quaternion::Identity();
    common::Vector3 last_torque_cmd_ = common::Vector3::Zero();
};

}  // namespace fsw
