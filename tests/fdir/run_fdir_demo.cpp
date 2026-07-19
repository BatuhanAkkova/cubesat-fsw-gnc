#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include "fsw/FlightSoftware.hpp"
#include "fsw/core/ModeManager.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"

#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimRW.hpp"

using namespace common;
using namespace fsw;
using namespace fsw::core;
using namespace fsw::gnc;
using namespace sim;

int main() {
    std::cout << "=== FDIR Actuator Fault Recovery Demo ===\n";

    // 1. Setup Simulation Environment
    // Spacecraft Inertia
    Matrix3 inertia = Matrix3::Identity() * 0.1;  // 0.1 kg*m^2

    // Initial state: Stationary at rest, identity attitude
    Quaternion q_init(1.0, 0.0, 0.0, 0.0);
    Vector3 w_init(0.0, 0.0, 0.0);
    auto body = std::make_unique<dynamics::RigidBody>(inertia, q_init, w_init);

    // Redundant reaction wheel config (4 wheels)
    std::vector<std::unique_ptr<SimRW>> wheels;
    SimRW::Config rw_cfg;
    rw_cfg.inertia = 0.001;     // 0.001 kg*m^2
    rw_cfg.max_torque = 0.05;   // 0.05 Nm
    rw_cfg.max_momentum = 0.1;  // 0.1 Nms
    rw_cfg.friction_coeff = 0.0001;
    rw_cfg.initial_speed = 0.0;

    for (int i = 0; i < 4; ++i) {
        wheels.push_back(std::make_unique<SimRW>(rw_cfg));
    }

    // 2. Setup Flight Software Configuration
    FlightSoftware::Config fsw_cfg;
    fsw_cfg.mode_cfg.safe_to_nominal_rate_threshold = 0.02;
    fsw_cfg.mode_cfg.nominal_to_safe_rate_threshold = 1.5;  // Allow smooth slew without SAFE mode trigger
    fsw_cfg.mode_cfg.min_time_in_mode = 1.0;
    fsw_cfg.sun_inertial = Vector3(1.0, 0.0, 0.0);  // Sun vector along +X

    // Build configuration JSON to initialize PID Attitude Controller
    nlohmann::json fsw_json;
    fsw_json["fsw"]["controllers"]["attitude"]["type"] = "PID";
    fsw_json["fsw"]["controllers"]["attitude"]["nominal_pid"]["kp"] = 0.2;
    fsw_json["fsw"]["controllers"]["attitude"]["nominal_pid"]["ki"] = 0.001;
    fsw_json["fsw"]["controllers"]["attitude"]["nominal_pid"]["kd"] = 1.8;
    fsw_json["fsw"]["controllers"]["attitude"]["nominal_pid"]["limit"] = 0.01;
    fsw_json["fsw"]["controllers"]["attitude"]["nominal_pid"]["anti_windup_limit"] = 0.005;
    fsw_cfg.full_config = fsw_json;

    FlightSoftware fsw(fsw_cfg);

    // Initial state setup: NOMINAL mode, Sun pointing target
    fsw.getModeManager().forceModeChange(MissionMode::NOMINAL, "Demo Start");

    // 3. Main Simulation Loop
    double dt = 0.1;          // 10Hz
    double duration = 120.0;  // 120 seconds
    int total_steps = static_cast<int>(duration / dt);

    std::ofstream csv("fdir_demo_data.csv");
    csv << "t,pointing_error,mode,w0_speed,w1_speed,w2_speed,w3_speed,w0_cmd,w1_cmd,w2_cmd,w3_cmd,w_norm\n";

    std::cout << "Running simulation for " << duration << " seconds...\n";
    std::cout << "Fault Injection (Stuck Wheel 0) at t = 30.0s.\n";

    std::vector<double> last_cmds(4, 0.0);

    for (int step = 0; step < total_steps; ++step) {
        double t = step * dt;

        // --- Fault Injection Stage ---
        if (t >= 30.0 && t < 30.05) {
            // Inject stuck fault on wheel 0: stuck at its current speed
            double current_speed = wheels[0]->getSpeed();
            wheels[0]->injectFault_Stuck(current_speed);
            std::cout << "[t=" << t << "s] FAULT INJECTED: Reaction Wheel 0 is STUCK at " << current_speed
                      << " rad/s\n";
        }

        // --- FSW Input Assembly ---
        SensorData sensors;
        sensors.q_measured = body->getAttitude();
        sensors.gyro_body = body->getAngularVelocity();
        // sun direction in body frame
        sensors.sun_body = body->getAttitude().conjugate() * fsw_cfg.sun_inertial;
        // Wheel speeds telemetry
        sensors.rw_speeds = {wheels[0]->getSpeed(), wheels[1]->getSpeed(), wheels[2]->getSpeed(),
                             wheels[3]->getSpeed()};
        sensors.rw_torques_prev = last_cmds;

        // --- FSW Step ---
        std::vector<std::vector<uint8_t>> raw_commands;
        fsw.step(sensors, raw_commands, dt);

        // Get reaction wheel commands from control allocation
        std::vector<double> rw_cmds = fsw.getRWTorqueCommands();
        if (rw_cmds.size() != 4) {
            rw_cmds.assign(4, 0.0);
        }
        last_cmds = rw_cmds;

        // --- SIM: Step Wheels ---
        // Apply torque commands to physical wheels and get reaction torque
        Vector3 internal_torque = Vector3::Zero();
        Vector3 internal_momentum = Vector3::Zero();

        // Tetrahedron/Pyramid wheel alignment axes
        double c = 0.7071067811865475;
        double s = 0.7071067811865475;
        Eigen::MatrixXd A(3, 4);
        A << c, 0.0, -c, 0.0, 0.0, c, 0.0, -c, s, s, s, s;

        Eigen::VectorXd u(4);
        for (int i = 0; i < 4; ++i) {
            wheels[i]->setTorqueCommand(rw_cmds[i]);
            double react_torque = wheels[i]->step(dt);
            u(i) = react_torque;  // SimRW::step returns -net_torque (reaction torque on body)
        }

        // Combine wheel reaction torques and momentum into body frame
        Eigen::Vector3d t_body = A * u;
        internal_torque = Vector3(t_body(0), t_body(1), t_body(2));

        Eigen::VectorXd h_w(4);
        for (int i = 0; i < 4; ++i) {
            h_w(i) = wheels[i]->getAngularMomentum();
        }
        Eigen::Vector3d h_body = A * h_w;
        internal_momentum = Vector3(h_body(0), h_body(1), h_body(2));

        // --- SIM: Step Spacecraft Dynamics ---
        Vector3 ext_torque = Vector3::Zero();
        if (t >= 30.0 && t <= 35.0) {
            ext_torque = Vector3(0.005, -0.003, 0.002);
        }
        body->step(dt, ext_torque, internal_torque, internal_momentum);

        // --- Log Telemetry ---
        // Calculate pointing error (angle between body Z and Sun inertial vector)
        Vector3 body_z = body->getAttitude() * Vector3(0.0, 0.0, 1.0);
        double cos_theta = body_z.dot(fsw_cfg.sun_inertial);
        double pointing_error = std::acos(std::clamp(cos_theta, -1.0, 1.0)) * 180.0 / PI;

        csv << t << "," << pointing_error << "," << static_cast<int>(fsw.getCurrentMode()) << ","
            << wheels[0]->getSpeed() << "," << wheels[1]->getSpeed() << "," << wheels[2]->getSpeed() << ","
            << wheels[3]->getSpeed() << "," << rw_cmds[0] << "," << rw_cmds[1] << "," << rw_cmds[2] << "," << rw_cmds[3]
            << "," << body->getAngularVelocity().norm() << "\n";

        if (step % 100 == 0) {
            std::cout << "[t=" << std::fixed << std::setprecision(1) << t
                      << "s] Mode=" << ModeManager::getModeString(fsw.getCurrentMode())
                      << " | Pointing Err=" << std::setprecision(2) << pointing_error << " deg"
                      << " | W0 Status=" << static_cast<int>(fsw.getFDIRManager().getWheelStatus(0)) << std::endl;
        }
    }

    csv.close();
    std::cout << "Simulation completed. Telemetry written to fdir_demo_data.csv\n";
    return 0;
}
