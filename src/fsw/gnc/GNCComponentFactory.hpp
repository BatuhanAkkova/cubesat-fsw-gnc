#pragma once

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/control/Bdot.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"
#include "fsw/gnc/interfaces/IController.hpp"
#include "fsw/gnc/interfaces/IEstimator.hpp"

#include "common/logger.hpp"

namespace fsw {
namespace gnc {

/**
 * @brief Factory class to create GNC components from JSON configuration.
 */
class GNCComponentFactory {
   public:
    using json = nlohmann::json;

    static std::unique_ptr<interfaces::IEstimator> createEstimator(const json& config) {
        if (!config.contains("type")) {
            common::LogError("[Factory] Estimator config missing 'type'. Defaulting to NULL.");
            return nullptr;
        }

        std::string type = config["type"];
        if (type == "MEKF") {
            auto mek_ptr = std::make_unique<ekf::MEKF>();

            // Optionally initialize with p0_diag/q_diag from config if present
            if (config.contains("p0_diag") && config["p0_diag"].is_array()) {
                common::MatrixX P0 = common::MatrixX::Identity(6, 6);
                for (int i = 0; i < 6; ++i) P0(i, i) = config["p0_diag"][i];
                mek_ptr->initialize(common::Quaternion::Identity(), common::Vector3::Zero(), P0);
            }

            return mek_ptr;
        }

        common::LogError("[Factory] Unknown Estimator type: {}", type);
        return nullptr;
    }

    static std::unique_ptr<interfaces::IController> createController(const json& config) {
        if (!config.contains("type")) {
            common::LogError("[Factory] Controller config missing 'type'.");
            return nullptr;
        }

        std::string type = config["type"];
        if (type == "PID") {
            control::AttitudeController::Config att_cfg;
            if (config.contains("nominal_pid")) {
                att_cfg.nominal_pid.kp = config["nominal_pid"]["kp"];
                att_cfg.nominal_pid.ki = config["nominal_pid"]["ki"];
                att_cfg.nominal_pid.kd = config["nominal_pid"]["kd"];
            }
            return std::make_unique<control::AttitudeController>(att_cfg);
        } else if (type == "Bdot") {
            double gain = config.value("gain", 1.0);
            return std::make_unique<control::Bdot>(gain);
        }

        common::LogError("[Factory] Unknown Controller type: {}", type);
        return nullptr;
    }
};

}  // namespace gnc
}  // namespace fsw
