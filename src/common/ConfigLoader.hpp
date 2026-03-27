#pragma once

#include <nlohmann/json.hpp>
#include <fstream>
#include "fsw/FlightSoftware.hpp"
#include "sim/Simulation.hpp"

namespace common {

/**
 * @brief Utility class to load FSW and Simulation configurations from JSON.
 */
class ConfigLoader {
public:
    using json = nlohmann::json;

    /**
     * @brief Load FlightSoftware::Config from a JSON file.
     */
    static fsw::FlightSoftware::Config loadFSWConfig(const std::string& path) {
        std::ifstream f(path);
        json data = json::parse(f);
        
        fsw::FlightSoftware::Config cfg;
        cfg.full_config = data; // Store full config for factory
        
        if (data.contains("fsw")) {
            auto fsw_json = data["fsw"];
            
            // Mode Transition Config (if present)
            if (fsw_json.contains("mode_manager")) {
                auto mm = fsw_json["mode_manager"];
                cfg.mode_cfg.safe_to_nominal_rate_threshold = mm.value("safe_to_nominal_threshold", 0.02);
                cfg.mode_cfg.nominal_to_safe_rate_threshold = mm.value("nominal_to_safe_threshold", 0.1);
                cfg.mode_cfg.min_time_in_mode = mm.value("min_time_in_mode", 10.0);
            }
        }

        return cfg;
    }

    /**
     * @brief Load Simulation::Config from a JSON file.
     */
    static sim::Simulation::Config loadSimConfig(const std::string& path) {
        std::ifstream f(path);
        json data = json::parse(f);

        sim::Simulation::Config cfg;
        auto sim_json = data["simulation"];

        if (sim_json.contains("inertia")) {
            auto inertia = sim_json["inertia"];
            cfg.inertia << inertia[0][0], inertia[0][1], inertia[0][2],
                           inertia[1][0], inertia[1][1], inertia[1][2],
                           inertia[2][0], inertia[2][1], inertia[2][2];
        }

        if (sim_json.contains("init_w")) {
            auto w = sim_json["init_w"];
            cfg.init_w << w[0], w[1], w[2];
        }

        return cfg;
    }
};

} // namespace common
