#include <fstream>
#include <iostream>

#include "fsw/FlightSoftware.hpp"

#include "common/ConfigLoader.hpp"
#include "common/logger.hpp"

#include "sim/Simulation.hpp"

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
        double duration = 100.0;  // Increased for better professional plotting
        int steps = static_cast<int>(duration / dt);

        std::ofstream csv("mission_data.csv");
        csv << "t,qw,qx,qy,qz,wx,wy,wz,error_deg\n";

        common::LogInfo("Starting simulation for {} seconds", duration);

        for (int i = 0; i < steps; ++i) {
            double t = i * dt;
            auto sensors = simulation.getSensors();
            std::vector<std::vector<uint8_t>> raw_commands;

            common::Vector3 torque_cmd = flight_software.step(sensors, raw_commands, dt);
            simulation.step(dt, torque_cmd);

            auto q = simulation.getAttitude();
            auto w = simulation.getAngularRate();

            // Calculate pointing error (simplified)
            common::Vector3 body_z = q * common::Vector3(0, 0, 1);
            common::Vector3 sun_inertial(1, 0, 0);
            double cos_theta = body_z.dot(sun_inertial);
            double error_deg = std::acos(std::clamp(cos_theta, -1.0, 1.0)) * 57.2958;

            csv << t << "," << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "," << w.x() << "," << w.y()
                << "," << w.z() << "," << error_deg << "\n";

            if (i % 100 == 0) {
                common::LogInfo("Step {}: t={:.1f}s | Pointing Error: {:.2f} deg", i, t, error_deg);
            }
        }

        csv.close();
        common::LogInfo("Simulation completed successfully. Data saved to mission_data.csv");

    } catch (const std::exception& e) {
        common::LogError("Error: {}", e.what());
        return 1;
    }

    return 0;
}
