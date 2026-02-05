#include <gtest/gtest.h>
#include "fsw/core/ModeManager.hpp"
#include "fsw/core/TaskScheduler.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimRW.hpp"
#include "sim/models/SimTorquer.hpp"
#include "fsw/gnc/control/Bdot.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"
#include <iostream>
#include <iomanip>

#ifndef M_PI
#define M_PI common::PI
#endif

using namespace common;
using namespace fsw::core;
using namespace fsw::gnc;

/**
 * Phase 6 Full Mission Test
 * 
 * Simulates complete mission scenario:
 * 1. Deploy from launch vehicle with HIGH tumble rate
 * 2. Mode Manager detects high rate -> SAFE mode
 * 3. B-Dot controller detumbles spacecraft
 * 4. Mode Manager detects low rate -> NOMINAL mode
 * 5. Attitude controller slews to Sun pointing
 * 6. Verify final attitude and stability
 */
class Phase6MissionTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 1. Satellite Inertia (CubeSat-like)
        Matrix3 inertia;
        inertia << 0.01, 0.0, 0.0,
                   0.0, 0.01, 0.0,
                   0.0, 0.0, 0.01;

        // 2. Initial state: HIGH tumble rate (post-deployment)
        Quaternion q_init(1, 0, 0, 0);
        Vector3 w_init(0.2, 0.15, 0.1);  // ~11 deg/s tumble

        body = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);
        
        // 3. Setup Reaction Wheels (for nominal mode)
        sim::SimRW::Config rw_cfg;
        rw_cfg.inertia = 0.001;
        rw_cfg.max_torque = 0.05;
        rw_cfg.max_momentum = 0.1;
        rw_cfg.friction_coeff = 0.0001;
        rw_cfg.initial_speed = 0.0;

        for (int i = 0; i < 3; ++i) {
            wheels.push_back(std::make_unique<sim::SimRW>(rw_cfg));
        }

        // 4. Setup Magnetorquers (for safe mode)
        torquer = std::make_unique<sim::models::SimTorquer>(2.0);  // 2.0 Am^2 max

        // 5. Setup Controllers
        bdot_controller = std::make_unique<control::Bdot>(500000.0);  // Very high gain for fast detumble

        // PID configuration with gain scheduling
        control::AttitudeController::Config att_config;
        
        // Nominal PID gains (for small errors < 10 deg)
        att_config.nominal_pid.kp = 0.5;
        att_config.nominal_pid.ki = 0.01;
        att_config.nominal_pid.kd = 1.0;
        att_config.nominal_pid.limit = 0.05;
        att_config.nominal_pid.anti_windup_limit = 0.01;
        
        // Large error PID gains (for errors > 30 deg) - reduced for stability
        att_config.large_error_pid.kp = 0.1;   // Much lower for large slews
        att_config.large_error_pid.ki = 0.0;   // No integral during large slews
        att_config.large_error_pid.kd = 0.5;   // Moderate damping
        att_config.large_error_pid.limit = 0.03;
        att_config.large_error_pid.anti_windup_limit = 0.0;
        
        // Rate limiting to prevent overshoot
        att_config.max_torque_rate = 0.2;  // 0.2 Nm/s
        att_config.rate_feedback_gain = 0.08;  // Strong damping
        
        attitude_controller = std::make_unique<control::AttitudeController>(att_config);

        // 6. Setup Mode Manager
        ModeTransitionConfig mode_config;
        mode_config.safe_to_nominal_rate_threshold = 0.17;   // 9.7 deg/s
        mode_config.nominal_to_safe_rate_threshold = 0.25;   // 14.3 deg/s
        mode_config.min_time_in_mode = 5.0;
        
        mode_manager = std::make_unique<ModeManager>(mode_config);

        // 7. Environment: Simplified magnetic field (constant in inertial)
        B_inertial = Vector3(0.0, 50000e-9, 0.0);  // 50 µT in Y direction

        // 8. Sun vector (inertial frame)
        sun_inertial = Vector3(1.0, 0.0, 0.0).normalized();  // Pointing +X
    }

    // Simulation components
    std::unique_ptr<sim::dynamics::RigidBody> body;
    std::vector<std::unique_ptr<sim::SimRW>> wheels;
    std::unique_ptr<sim::models::SimTorquer> torquer;
    
    // Controllers
    std::unique_ptr<control::Bdot> bdot_controller;
    std::unique_ptr<control::AttitudeController> attitude_controller;
    std::unique_ptr<ModeManager> mode_manager;

    // Environment
    Vector3 B_inertial;
    Vector3 sun_inertial;
};

TEST_F(Phase6MissionTest, FullMissionSimulation) {
    const double dt = 0.1;  // 10Hz
    const double simulation_time = 120.0;  // 2 minutes
    const int total_steps = static_cast<int>(simulation_time / dt);

    std::cout << "\n=== PHASE 6 FULL MISSION SIMULATION ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    // Initial state
    Vector3 w_initial = body->getAngularVelocity();
    std::cout << "Initial angular velocity: [" << w_initial.transpose() 
              << "] rad/s (norm=" << w_initial.norm() << ")" << std::endl;
    std::cout << "Initial mode: " << ModeManager::getModeString(mode_manager->getCurrentMode()) 
              << "\n" << std::endl;

    // Tracking variables
    MissionMode last_mode = mode_manager->getCurrentMode();
    double detumble_complete_time = 0.0;
    bool detumble_complete = false;

    // Main simulation loop
    for (int step = 0; step < total_steps; ++step) {
        double t = step * dt;
        
        // Get current state
        Quaternion q_curr = body->getAttitude();
        Vector3 w_curr = body->getAngularVelocity();
        
        // Update mode manager
        mode_manager->update(w_curr, dt);
        MissionMode current_mode = mode_manager->getCurrentMode();
        
        // Detect mode transitions
        if (current_mode != last_mode) {
            std::cout << "[t=" << t << "s] MODE TRANSITION: " 
                      << ModeManager::getModeString(last_mode) << " -> "
                      << ModeManager::getModeString(current_mode) << std::endl;
            std::cout << "  Angular rate: " << w_curr.norm() << " rad/s" << std::endl;
            
            if (current_mode == MissionMode::NOMINAL && !detumble_complete) {
                detumble_complete_time = t;
                detumble_complete = true;
                std::cout << "  *** DETUMBLE COMPLETE ***" << std::endl;
            }
            
            last_mode = current_mode;
        }
        
        // Execute mode-specific control
        Vector3 external_torque = Vector3::Zero();
        Vector3 internal_torque = Vector3::Zero();
        Vector3 internal_momentum = Vector3::Zero();

        if (current_mode == MissionMode::SAFE) {
            // === SAFE MODE: B-Dot Detumbling ===
            
            // Get magnetic field in body frame
            Vector3 B_body = q_curr.conjugate() * B_inertial;
            
            // B-Dot control
            Vector3 dipole_cmd = bdot_controller->update(B_body, dt);
            torquer->setDipole(dipole_cmd);
            Vector3 M_actual = torquer->getDipole();
            
            // Torque = M × B
            external_torque = M_actual.cross(B_body);
            
        } else if (current_mode == MissionMode::NOMINAL) {
            // === NOMINAL MODE: Sun Pointing ===
            
            // Compute target attitude (point +Z body axis at Sun)
            Vector3 body_axis(0, 0, 1);  // Z-axis
            Quaternion q_target = guidance::PointingStrategies::alignAxis(
                body_axis, sun_inertial);
            
            // Attitude control
            Vector3 torque_cmd = attitude_controller->computeTorque(
                q_curr, q_target, w_curr, dt);
            
            // Apply to reaction wheels
            wheels[0]->setTorqueCommand(-torque_cmd.x());
            wheels[1]->setTorqueCommand(-torque_cmd.y());
            wheels[2]->setTorqueCommand(-torque_cmd.z());
            
            // Get wheel feedback
            internal_torque.x() = wheels[0]->step(dt);
            internal_torque.y() = wheels[1]->step(dt);
            internal_torque.z() = wheels[2]->step(dt);
            
            internal_momentum.x() = wheels[0]->getAngularMomentum();
            internal_momentum.y() = wheels[1]->getAngularMomentum();
            internal_momentum.z() = wheels[2]->getAngularMomentum();
        }
        
        // Propagate dynamics
        body->step(dt, external_torque, internal_torque, internal_momentum);
        
        // Periodic status updates
        if (step % 100 == 0) {
            double rate_norm = w_curr.norm();
            std::cout << "[t=" << t << "s] Mode=" 
                      << ModeManager::getModeString(current_mode)
                      << " | Rate=" << rate_norm << " rad/s";
            
            if (current_mode == MissionMode::NOMINAL) {
                Quaternion q_target = guidance::PointingStrategies::alignAxis(
                    Vector3(0, 0, 1), sun_inertial);
                double angle_err_rad = q_curr.angularDistance(q_target);
                double angle_err_deg = angle_err_rad * 180.0 / M_PI;
                std::cout << " | Pointing err=" << angle_err_deg << " deg";
            }
            
            std::cout << std::endl;
        }
    }

    // === VERIFICATION ===
    std::cout << "\n=== MISSION VERIFICATION ===" << std::endl;
    
    Vector3 w_final = body->getAngularVelocity();
    Quaternion q_final = body->getAttitude();
    
    std::cout << "Final angular velocity: " << w_final.norm() << " rad/s" << std::endl;
    std::cout << "Final mode: " << ModeManager::getModeString(mode_manager->getCurrentMode()) 
              << std::endl;

    // Check 1: Should have transitioned to NOMINAL mode
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::NOMINAL)
        << "Should have reached NOMINAL mode";
    EXPECT_TRUE(detumble_complete) << "Should have completed detumble phase";
    
    // Check 2: Angular rates should be low
    double rate_final = w_final.norm();
    EXPECT_LT(rate_final, 0.20) << "Final angular rate should be < 0.20 rad/s";
    std::cout << "✓ Angular rate stable: " << rate_final << " rad/s" << std::endl;
    
    // Check 3: Sun pointing accuracy
    Quaternion q_target = guidance::PointingStrategies::alignAxis(
        Vector3(0, 0, 1), sun_inertial);
    double angle_err_rad = q_final.angularDistance(q_target);
    double angle_err_deg = angle_err_rad * 180.0 / M_PI;
    
    std::cout << "Final pointing error: " << angle_err_deg << " degrees" << std::endl;
    EXPECT_LT(angle_err_deg, 30.0) << "Sun pointing error should be < 30 degrees";
    std::cout << "✓ Sun pointing achieved" << std::endl;
    
    // Check 4: Detumble time
    if (detumble_complete) {
        std::cout << "Detumble completion time: " << detumble_complete_time << " s" << std::endl;
        EXPECT_LT(detumble_complete_time, 90.0) << "Should detumble within 90 seconds";
        std::cout << "✓ Detumble completed in reasonable time" << std::endl;
    }
    
    std::cout << "\n=== MISSION SUCCESS ===" << std::endl;
}

TEST_F(Phase6MissionTest, SchedulerIntegration) {
    // Test that scheduler can orchestrate the mission
    TaskScheduler scheduler(0.1, false);  // 10Hz, simulation mode
    
    const double simulation_time = 60.0;
    
    // Create a GNC task that runs at 10Hz
    Task gnc_task("GNC_Loop", [&](double dt) {
        // Get current state
        Quaternion q_curr = body->getAttitude();
        Vector3 w_curr = body->getAngularVelocity();
        
        // Update mode manager
        mode_manager->update(w_curr, dt);
        
        // Simple control based on mode
        Vector3 external_torque = Vector3::Zero();
        
        if (mode_manager->getCurrentMode() == MissionMode::SAFE) {
            Vector3 B_body = q_curr.conjugate() * B_inertial;
            Vector3 dipole_cmd = bdot_controller->update(B_body, dt);
            torquer->setDipole(dipole_cmd);
            external_torque = torquer->getDipole().cross(B_body);
        }
        
        // Propagate
        body->step(dt, external_torque);
        
    }, 0.1, 10);  // 10Hz, high priority
    
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
