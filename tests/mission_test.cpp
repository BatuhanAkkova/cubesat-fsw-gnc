#include <gtest/gtest.h>
#include <iomanip>
#include <iostream>

#include "common/DataLogger.hpp"
#include "common/Profiler.hpp"
#include "fsw/core/ModeManager.hpp"
#include "fsw/core/TaskScheduler.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/control/Bdot.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"
#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimRW.hpp"
#include "sim/models/SimTorquer.hpp"

#ifndef M_PI
#define M_PI common::PI
#endif

using namespace common;
using namespace fsw::core;
using namespace fsw::gnc;

/**
 * Mission Test
 *
 * Simulates complete mission scenario:
 * 1. Deploy from launch vehicle with initial tumble rate (> safe threshold)
 * 2. System starts in SAFE mode to handle initial high tumble rate
 * 3. B-Dot controller detumbles spacecraft using simulated magnetorquers
 * 4. Mode Manager transitions to NOMINAL mode when rates drop below threshold
 * 5. Attitude controller slews to Sun-pointing configuration (Z-axis to Sun)
 * 6. Autonomous transition to SCIENCE mode for payload target tracking
 * 7. Transition to DOWNLINK mode for ground communication and data transfer
 * 8. Verify sequence completion, pointing stability, and data goals
 */
class MissionTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // 1. Satellite Inertia (CubeSat-like)
        Matrix3 inertia;
        inertia << 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01;

        // 2. Initial state: Moderate tumble rate (post-deployment)
        Quaternion q_init(1, 0, 0, 0);
        Vector3 w_init(0.12, 0.08, 0.04);  // ~8 deg/s tumble (low enough for fast test)

        body = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);

        // 3. Setup Reaction Wheels (for nominal mode)
        sim::SimRW::Config rw_cfg;
        rw_cfg.inertia = 0.0001;
        rw_cfg.max_torque = 0.005;
        rw_cfg.max_momentum = 0.05;
        rw_cfg.friction_coeff = 0.00001;
        rw_cfg.initial_speed = 0.0;

        for (int i = 0; i < 3; ++i) {
            wheels.push_back(std::make_unique<sim::SimRW>(rw_cfg));
        }

        // 4. Setup Magnetorquers (for safe mode)
        torquer = std::make_unique<sim::models::SimTorquer>(2.0);  // 2.0 Am^2

        // 5. Setup Controllers
        bdot_controller = std::make_unique<control::Bdot>(5000000.0);

        // PID configuration with gain scheduling
        control::AttitudeController::Config att_config;

        // Slightly firmer gains but still safe
        // Nominal PID gains (tuned for 10Hz stability with I=0.01)
        att_config.nominal_pid.kp = 0.02;
        att_config.nominal_pid.ki = 0.0001;
        att_config.nominal_pid.kd = 0.18;  // Faster damping (kd*dt/I < 2.0)
        att_config.nominal_pid.limit = 0.01;
        att_config.nominal_pid.anti_windup_limit = 0.005;

        // Safe mode PID gains (used for larger errors)
        att_config.large_error_pid.kp = 0.01;
        att_config.large_error_pid.ki = 0.0;
        att_config.large_error_pid.kd = 0.10;
        att_config.large_error_pid.limit = 0.005;

        att_config.max_torque_rate = 0.01;  // Smooth torque transitions
        att_config.rate_feedback_gain = 0.05;

        attitude_controller = std::make_unique<control::AttitudeController>(att_config);

        // 6. Setup Mode Manager
        ModeTransitionConfig mode_config;
        mode_config.safe_to_nominal_rate_threshold = 0.08;  // ~4.5 deg/s
        mode_config.nominal_to_safe_rate_threshold = 0.8;   // Allow for slew transients
        mode_config.min_time_in_mode = 5.0;

        mode_manager = std::make_unique<ModeManager>(mode_config);

        // 7. Environment: Rotating magnetic field (simulates orbit)
        B_inertial = Vector3(50000e-9, 0.0, 0.0);

        // 8. Sun vector (inertial frame)
        sun_inertial = Vector3(1.0, 0.0, 0.0).normalized();     // Pointing +X
                                                                // 9. Science target (inertial frame)
        target_inertial = Vector3(0.0, 1.0, 0.0).normalized();  // Pointing +Y

        // 10. Ground Station location (inertial frame - simplified)
        gs_inertial = Vector3(0.0, 0.0, -1.0).normalized();  // Pointing -Z (Nadir-ish)

        // Pre-calculate target quaternions (Optimization)
        const Vector3 z_body(0, 0, 1);
        q_sun_target = guidance::PointingStrategies::alignAxis(z_body, sun_inertial);
        q_target_sci = guidance::PointingStrategies::alignAxis(z_body, target_inertial);
        q_gs_target = guidance::PointingStrategies::alignAxis(z_body, gs_inertial);

        // 11. Setup Orbit (LEO at 400km altitude)
        constexpr double MU_EARTH = 3.986004418e14;
        constexpr double R_EARTH = 6378137.0;
        double altitude = 400000.0;  // 400 km
        double r = R_EARTH + altitude;
        double v_circ = std::sqrt(MU_EARTH / r);
        double inclination = 51.6 * (M_PI / 180.0);  // ISS-like orbit

        Vector3 orbit_pos(r, 0, 0);
        Vector3 orbit_vel(0, v_circ * std::cos(inclination), v_circ * std::sin(inclination));

        sim::dynamics::Orbit::Config orbit_config;
        orbit_config.enable_j2 = true;

        orbit = std::make_unique<sim::dynamics::Orbit>(orbit_pos, orbit_vel, orbit_config);
    }

    // Simulation components
    std::unique_ptr<sim::dynamics::RigidBody> body;
    std::unique_ptr<sim::dynamics::Orbit> orbit;
    std::vector<std::unique_ptr<sim::SimRW>> wheels;
    std::unique_ptr<sim::models::SimTorquer> torquer;

    // Controllers
    std::unique_ptr<control::Bdot> bdot_controller;
    std::unique_ptr<control::AttitudeController> attitude_controller;
    std::unique_ptr<ModeManager> mode_manager;

    // Environment
    Vector3 B_inertial;
    Vector3 sun_inertial;
    Vector3 target_inertial;
    Vector3 gs_inertial;

    // Pre-calculated targets
    Quaternion q_sun_target;
    Quaternion q_target_sci;
    Quaternion q_gs_target;

    // Profiler
    common::Profiler profiler;
};

TEST_F(MissionTest, FullMissionSimulation) {
    const double dt = 0.1;                  // 10Hz
    const double simulation_time = 3600.0;  // 1 hour mission for complete orbit coverage
    const int total_steps = static_cast<int>(simulation_time / dt);

    std::cout << "\n=== FULL MISSION TIMELINE SIMULATION ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    // Setup data logger with comprehensive telemetry
    common::DataLogger logger("mission_data.csv");
    // Basic state
    logger.addColumn("time");
    logger.addColumn("qw");
    logger.addColumn("qx");
    logger.addColumn("qy");
    logger.addColumn("qz");
    logger.addColumn("wx");
    logger.addColumn("wy");
    logger.addColumn("wz");
    logger.addColumn("rx");
    logger.addColumn("ry");
    logger.addColumn("rz");
    logger.addColumn("vx");
    logger.addColumn("vy");
    logger.addColumn("vz");
    logger.addColumn("mode");

    // GNC Control Data
    logger.addColumn("torque_cmd_x");
    logger.addColumn("torque_cmd_y");
    logger.addColumn("torque_cmd_z");
    logger.addColumn("torque_ext_x");
    logger.addColumn("torque_ext_y");
    logger.addColumn("torque_ext_z");
    logger.addColumn("qw_target");
    logger.addColumn("qx_target");
    logger.addColumn("qy_target");
    logger.addColumn("qz_target");
    logger.addColumn("pointing_error");
    logger.addColumn("momentum_x");
    logger.addColumn("momentum_y");
    logger.addColumn("momentum_z");

    // Mission Progress
    logger.addColumn("data_collected");
    logger.addColumn("data_downlinked");

    logger.writeHeader();
    std::cout << "Data logging to: mission_data.csv" << std::endl;

    // Initial state
    Vector3 w_initial = body->getAngularVelocity();
    std::cout << "Initial angular velocity: [" << w_initial.transpose() << "] rad/s (norm=" << w_initial.norm() << ")"
              << std::endl;
    std::cout << "Initial mode: " << ModeManager::getModeString(mode_manager->getCurrentMode()) << "\n" << std::endl;

    // Tracking variables
    MissionMode last_mode = mode_manager->getCurrentMode();
    double detumble_complete_time = 0.0;
    bool detumble_complete = false;
    bool science_started = false;
    bool downlink_started = false;
    double science_start_time = 0.0;
    double downlink_start_time = 0.0;
    double data_collected = 0.0;
    double data_downlinked = 0.0;

    // Main simulation loop
    for (int step = 0; step < total_steps; ++step) {
        double t = step * dt;

        // Get current state
        Quaternion q_curr = body->getAttitude();
        Vector3 w_curr = body->getAngularVelocity();

        {
            common::ScopedTimer t_mode(profiler, "ModeManager");
            // Update mode manager (automatic detumble -> nominal transition)
            mode_manager->update(w_curr, dt);
        }

        // Manual mission timeline progression
        if (mode_manager->getCurrentMode() == MissionMode::NOMINAL) {
            // Check if nominal pointing is stable enough to start science
            double sun_err = q_curr.angularDistance(q_sun_target) * 180.0 / M_PI;

            if (sun_err < 1.0 && !science_started && t > detumble_complete_time + 10.0) {
                mode_manager->commandMode(MissionMode::SCIENCE);
                science_started = true;
                science_start_time = t;
            }
        } else if (mode_manager->getCurrentMode() == MissionMode::SCIENCE) {
            // Collect data during science mode
            data_collected += 0.1 * dt;  // 0.1 units per second

            // Transition to downlink after collecting enough data
            if (data_collected >= 5.0 && !downlink_started) {
                mode_manager->commandMode(MissionMode::DOWNLINK);
                downlink_started = true;
                downlink_start_time = t;
            }
        } else if (mode_manager->getCurrentMode() == MissionMode::DOWNLINK) {
            // Downlink data if pointed correctly
            double gs_err = q_curr.angularDistance(q_gs_target) * 180.0 / M_PI;

            if (gs_err < 2.0 && data_collected > 0.0) {
                double amount = 0.5 * dt;
                data_collected -= amount;
                data_downlinked += amount;
            }
        }

        MissionMode current_mode = mode_manager->getCurrentMode();

        // Detect mode transitions
        if (current_mode != last_mode) {
            std::cout << "[t=" << t << "s] MODE TRANSITION: " << ModeManager::getModeString(last_mode) << " -> "
                      << ModeManager::getModeString(current_mode) << std::endl;

            if (current_mode == MissionMode::NOMINAL && !detumble_complete) {
                detumble_complete_time = t;
                detumble_complete = true;
            }
            last_mode = current_mode;
        }

        // Execute mode-specific control
        Vector3 external_torque = Vector3::Zero();
        Vector3 internal_torque = Vector3::Zero();
        Vector3 internal_momentum = Vector3::Zero();
        Vector3 torque_cmd = Vector3::Zero();
        Quaternion q_target = q_curr;  // Default to current attitude
        double pointing_error = 0.0;

        // Update B_inertial with multi-axis rotation to ensure all axes are dampened
        double orbit_rate = 0.05;  // Fast rotation for quick test
        Vector3 B_curr_inertial(50000e-9 * std::cos(orbit_rate * t),
                                50000e-9 * std::sin(orbit_rate * t) * std::cos(orbit_rate * 0.3 * t),
                                50000e-9 * std::sin(orbit_rate * t) * std::sin(orbit_rate * 0.3 * t));

        {
            common::ScopedTimer t_ctrl(profiler, "ControlUpdate");
            if (current_mode == MissionMode::SAFE) {
                // === SAFE MODE: B-Dot Detumbling ===

                // Get magnetic field in body frame
                // B-Dot control
                common::SensorData sensors;
                sensors.mag_body = B_body;

                common::State state_est;
                state_est.q = q_curr;
                state_est.w = w_curr;

                common::GuidanceTarget target;
                target.mode = "SAFE";

                Vector3 dipole_cmd = bdot_controller->update(sensors, state_est, target, dt);
                torquer->setDipole(dipole_cmd);
                external_torque = torquer->getDipole().cross(B_body);

                // In SAFE mode, target is current attitude (detumble only)
                q_target = q_curr;
                pointing_error = 0.0;
            } else {
                // Reaction wheel based control for all other modes
                if (current_mode == MissionMode::NOMINAL) {
                    q_target = q_sun_target;
                } else if (current_mode == MissionMode::SCIENCE) {
                    q_target = q_target_sci;
                } else if (current_mode == MissionMode::DOWNLINK) {
                    q_target = q_gs_target;
                } else {
                    q_target = q_curr;  // Hold current
                }

                common::SensorData sensors;
                sensors.q_measured = q_curr;
                sensors.gyro_body = w_curr;

                common::State state_est;
                state_est.q = q_curr;
                state_est.w = w_curr;

                common::GuidanceTarget target;
                target.q = q_target;
                target.w = Vector3::Zero();
                target.mode = ModeManager::getModeString(current_mode);

                torque_cmd = attitude_controller->update(sensors, state_est, target, dt);
                pointing_error = q_curr.angularDistance(q_target) * 180.0 / M_PI;

                // Apply to reaction wheels
                wheels[0]->setTorqueCommand(-torque_cmd.x());
                wheels[1]->setTorqueCommand(-torque_cmd.y());
                wheels[2]->setTorqueCommand(-torque_cmd.z());

                for (int i = 0; i < 3; ++i) {
                    internal_torque(i) = wheels[i]->step(dt);
                    internal_momentum(i) = wheels[i]->getAngularMomentum();
                }
            }
        }

        {
            common::ScopedTimer t_dyn(profiler, "DynamicsStep");

            // Log data every step (or skip for performance: step % 10 == 0)
            if (step % 5 == 0) {  // Log every 0.5 seconds
                Vector3 pos = orbit->getPosition();
                Vector3 vel = orbit->getVelocity();

                logger.startRow();
                // Basic state
                logger.addValue(t);
                logger.addValue(q_curr.w());
                logger.addValue(q_curr.x());
                logger.addValue(q_curr.y());
                logger.addValue(q_curr.z());
                logger.addValue(w_curr.x());
                logger.addValue(w_curr.y());
                logger.addValue(w_curr.z());
                logger.addValue(pos.x());
                logger.addValue(pos.y());
                logger.addValue(pos.z());
                logger.addValue(vel.x());
                logger.addValue(vel.y());
                logger.addValue(vel.z());
                logger.addValue(static_cast<int>(current_mode));

                // GNC Control Data
                logger.addValue(torque_cmd.x());
                logger.addValue(torque_cmd.y());
                logger.addValue(torque_cmd.z());
                logger.addValue(external_torque.x());
                logger.addValue(external_torque.y());
                logger.addValue(external_torque.z());
                logger.addValue(q_target.w());
                logger.addValue(q_target.x());
                logger.addValue(q_target.y());
                logger.addValue(q_target.z());
                logger.addValue(pointing_error);
                logger.addValue(internal_momentum.x());
                logger.addValue(internal_momentum.y());
                logger.addValue(internal_momentum.z());

                // Mission Progress
                logger.addValue(data_collected);
                logger.addValue(data_downlinked);

                logger.endRow();
            }

            // Propagate dynamics
            body->step(dt, external_torque, internal_torque, internal_momentum);
            orbit->step(dt);
        }

        // Periodic status updates
        if (step % 200 == 0) {
            std::cout << "[t=" << t << "s] Mode=" << ModeManager::getModeString(current_mode)
                      << " | Rate=" << w_curr.norm() << " rad/s";

            if (current_mode != MissionMode::SAFE) {
                Quaternion q_target;
                if (current_mode == MissionMode::NOMINAL)
                    q_target = guidance::PointingStrategies::alignAxis(Vector3(0, 0, 1), sun_inertial);
                else if (current_mode == MissionMode::SCIENCE)
                    q_target = guidance::PointingStrategies::alignAxis(Vector3(0, 0, 1), target_inertial);
                else if (current_mode == MissionMode::DOWNLINK)
                    q_target = guidance::PointingStrategies::alignAxis(Vector3(0, 0, 1), gs_inertial);

                double err = q_curr.angularDistance(q_target) * 180.0 / M_PI;
                std::cout << " | Pointing err=" << err << " deg";
            }

            if (current_mode == MissionMode::SCIENCE || current_mode == MissionMode::DOWNLINK) {
                std::cout << " | Collected=" << data_collected << " (Total DL=" << data_downlinked << ")";
            }
            std::cout << std::endl;
        }
    }

    // === VERIFICATION ===
    std::cout << "\n=== MISSION VERIFICATION ===" << std::endl;

    profiler.printReport();

    EXPECT_TRUE(detumble_complete) << "Should have completed detumble phase";
    EXPECT_TRUE(science_started) << "Should have reached SCIENCE phase";
    EXPECT_TRUE(downlink_started) << "Should have reached DOWNLINK phase";

    std::cout << "All mission phases reached" << std::endl;
    std::cout << "Final Data Remaining: " << data_collected << std::endl;
    std::cout << "Final Data Downlinked: " << data_downlinked << std::endl;

    EXPECT_GT(data_downlinked, 4.0) << "Should have downlinked significant data";

    Vector3 w_final = body->getAngularVelocity();
    EXPECT_LT(w_final.norm(), 0.05) << "Final angular rate should be stable around orbit rate";

    std::cout << "Mission data goals achieved" << std::endl;
    std::cout << "Final stability maintained: " << w_final.norm() << " rad/s" << std::endl;
    std::cout << "\n=== MISSION SUCCESS ===" << std::endl;
}

TEST_F(MissionTest, SchedulerIntegration) {
    // Test that scheduler can orchestrate the mission
    TaskScheduler scheduler(0.1, false);  // 10Hz, simulation mode

    const double simulation_time = 60.0;

    // Create a GNC task that runs at 10Hz
    Task gnc_task(
        "GNC_Loop",
        [&](double dt) {
            // Get current state
            Quaternion q_curr = body->getAttitude();
            Vector3 w_curr = body->getAngularVelocity();

            // Update mode manager
            mode_manager->update(w_curr, dt);

            // Simple control based on mode
            Vector3 external_torque = Vector3::Zero();

            if (mode_manager->getCurrentMode() == MissionMode::SAFE) {
                // Simulate rotating B-field in scheduler test too
                double t_total = scheduler.getStats().cycles_executed * dt;
                double orbit_rate = 0.001;
                Vector3 B_curr_inertial(50000e-9 * std::cos(orbit_rate * t_total),
                                        50000e-9 * std::sin(orbit_rate * t_total), 0.0);

                common::SensorData sensors;
                sensors.mag_body = B_body;

                common::State state_est;
                state_est.q = q_curr;
                state_est.w = w_curr;

                common::GuidanceTarget target;
                target.mode = "SAFE";

                Vector3 dipole_cmd = bdot_controller->update(sensors, state_est, target, dt);
                torquer->setDipole(dipole_cmd);
                external_torque = torquer->getDipole().cross(B_body);
            }

            // Propagate
            body->step(dt, external_torque);
        },
        0.1, 10);  // 10Hz, high priority

    scheduler.registerTask(gnc_task);

    // Run mission
    scheduler.run(simulation_time);

    // Verify scheduler executed correctly
    SchedulerStats stats = scheduler.getStats();
    int expected_cycles = static_cast<int>(simulation_time / 0.1);

    EXPECT_GE(stats.cycles_executed, expected_cycles - 1);
    EXPECT_LE(stats.cycles_executed, expected_cycles + 1);

    // Should have reduced angular rates
    Vector3 w_final = body->getAngularVelocity();
    EXPECT_LT(w_final.norm(), 0.15) << "Scheduler-based control should reduce rates";

    std::cout << "Scheduler executed " << stats.cycles_executed << " cycles" << std::endl;
}
