#include <iostream>
#include "common/ConfigLoader.hpp"
#include "sim/Simulation.hpp"
#include "fsw/FlightSoftware.hpp"
#include "common/logger.hpp"

int main(int argc, char* argv[]) {
    std::string config_path = "sample_config.json";
    if (argc > 1) {
        config_path = argv[1];
    }

    try {
        common::Logger::Init();
        common::LogInfo("Loading configuration from: {}", config_path);
        auto fsw_cfg = common::ConfigLoader::loadFSWConfig(config_path);
        auto sim_cfg = common::ConfigLoader::loadSimConfig(config_path);

        sim::Simulation simulation(sim_cfg);
        fsw::FlightSoftware flight_software(fsw_cfg);

        double dt = 0.1;
        double duration = 10.0;
        int steps = static_cast<int>(duration / dt);

        common::LogInfo("Starting simulation for {} seconds", duration);

        for (int i = 0; i < steps; ++i) {
            auto sensors = simulation.getSensors();
            
            // In a real system, raw_commands would come from a telemetry link
            std::vector<std::vector<uint8_t>> raw_commands; 
            
            common::Vector3 torque_cmd = flight_software.step(sensors, raw_commands, dt);
            
            simulation.step(dt, torque_cmd);

            if (i % 10 == 0) {
                auto q = simulation.getAttitude();
                common::LogInfo("Step {}: Attitude Q=[{:.4f}, {:.4f}, {:.4f}, {:.4f}]", 
                         i, q.w(), q.x(), q.y(), q.z());
            }
        }

        common::LogInfo("Simulation completed successfully.");

    } catch (const std::exception& e) {
        common::LogError("Error: {}", e.what());
        return 1;
    }

    return 0;
}
