/**
 * @file wheel_desaturation_demo.cpp
 * @brief Demonstration of automatic wheel desaturation using magnetorquers
 * 
 * This demo shows:
 * - Reaction wheels saturating during pointing maneuvers
 * - Automatic detection of high momentum
 * - B-cross-H control law desaturating wheels
 * - State machine transitions (IDLE -> DESATURATING -> COOLDOWN)
 */

#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimMagnetometer.hpp"
#include "sim/models/SimRW.hpp"
#include "sim/models/SimTorquer.hpp"
#include "fsw/gnc/control/WheelDesaturation.hpp"
#include <iostream>
#include <iomanip>
#include <memory>
#include <vector>

using namespace sim;
using namespace sim::dynamics;
using namespace sim::models;
using namespace fsw::gnc::control;

int main() {
    std::cout << "=== Wheel Desaturation Demonstration ===\n\n";
    
    // === Setup ===
    std::cout << "Setting up simulation...\n";
    
    // Orbit (400 km LEO, 51.6 deg inclination)
    double altitude = 400000.0;
    double r = 6378137.0 + altitude;
    double v_circ = std::sqrt(3.986004418e14 / r);
    double inclination = 51.6 * 3.14159265358979323846 / 180.0;
    
    common::Vector3 init_pos(r, 0, 0);
    common::Vector3 init_vel(0, v_circ * std::cos(inclination), v_circ * std::sin(inclination));
    
    Orbit::Config orbit_config;
    orbit_config.enable_j2 = true;
    Orbit orbit(init_pos, init_vel, orbit_config);
    
    // Rigid Body
    common::Matrix3 inertia = common::Matrix3::Identity() * 0.1;
    common::Quaternion init_attitude = common::Quaternion::Identity();
    common::Vector3 init_omega = common::Vector3::Zero();
    RigidBody body(inertia, init_attitude, init_omega);
    
    // Magnetometer with Earth rotation
    SimMagnetometer::Config mag_config;
    mag_config.enable_earth_rotation = true;
    mag_config.use_tilted_dipole = false;
    SimMagnetometer magnetometer(body, orbit, mag_config);
    
    // Reaction Wheels (3-axis)
    std::vector<std::shared_ptr<SimRW>> wheels;
    SimRW::Config rw_config;
    rw_config.inertia = 0.001;          // 0.001 kg*m^2
    rw_config.max_torque = 0.01;        // 0.01 Nm
    rw_config.max_momentum = 0.05;      // 0.05 Nms
    rw_config.friction_coeff = 0.0;
    rw_config.initial_speed = 0.0;
    
    for (int i = 0; i < 3; i++) {
        wheels.push_back(std::make_shared<SimRW>(rw_config));
    }
    
    // Magnetorquers (3-axis)
    std::vector<std::shared_ptr<SimTorquer>> torquers;
    for (int i = 0; i < 3; i++) {
        torquers.push_back(std::make_shared<SimTorquer>(0.2));  // 0.2 A*m^2 max dipole
    }
    
    // Wheel Desaturation Controller
    std::vector<std::shared_ptr<hal::IRW>> wheels_interface(wheels.begin(), wheels.end());
    WheelDesaturation::Config desat_config;
    desat_config.momentum_threshold_ratio = 0.75;  // Trigger at 75%
    desat_config.desat_gain = 5e-7;                // Control gain
    desat_config.max_desat_duration = 300.0;       // 5 min max
    desat_config.min_time_between_desat = 60.0;    // 1 min cooldown
    WheelDesaturation desaturator(desat_config);
    
    std::cout << "Simulation setup complete.\n\n";
    
    // === Phase 1: Saturate Wheels ===
    std::cout << "Phase 1: Saturating wheels with constant torque...\n";
    std::cout << "Time(s) | Wheel Momentum (Nms) | Status\n";
    std::cout << "--------|----------------------|--------\n";
    
    double dt = 1.0;
    double sim_time = 0.0;
    
    // Apply torque to saturate wheels
    wheels[0]->setTorqueCommand(0.008);
    wheels[1]->setTorqueCommand(0.007);
    wheels[2]->setTorqueCommand(0.006);
    
    for (int i = 0; i < 60; i++) {
        for (auto& wheel : wheels) {
            wheel->step(dt);
        }
        
        if (i % 10 == 0) {
            double h_total = 0.0;
            for (auto& wheel : wheels) {
                h_total += std::abs(wheel->getAngularMomentum());
            }
            std::cout << std::setw(7) << i << " | "
                      << std::setw(20) << std::fixed << std::setprecision(4) << h_total
                      << " | Saturating\n";
        }
    }
    
    double h_total_saturated = 0.0;
    for (auto& wheel : wheels) {
        h_total_saturated += std::abs(wheel->getAngularMomentum());
    }
    std::cout << "\nWheels saturated to " << h_total_saturated << " Nms ("
              << (h_total_saturated / (3 * 0.05) * 100.0) << "% of max)\n\n";
    
    // === Phase 2: Automatic Desaturation ===
    std::cout << "Phase 2: Automatic desaturation...\n";
    std::cout << "Time(s) | Momentum(Nms) | B-field(μT) | Dipole(Am²) | State\n";
    std::cout << "--------|---------------|-------------|-------------|------------\n";
    
    // Stop commanding torque
    for (auto& wheel : wheels) {
        wheel->setTorqueCommand(0.0);
    }
    
    for (int i = 0; i < 300; i++) {  // 5 minutes
        sim_time += dt;
        magnetometer.setTime(sim_time);
        
        // Propagate dynamics
        orbit.step(dt);
        
        // Read B-field
        common::Vector3 b_field = magnetometer.read();
        
        // Update desaturation controller
        common::Vector3 dipole_cmd = desaturator.update(wheels_interface, b_field, dt);
        
        // Command magnetorquers
        torquers[0]->setDipole(common::Vector3(dipole_cmd.x(), 0, 0));
        torquers[1]->setDipole(common::Vector3(0, dipole_cmd.y(), 0));
        torquers[2]->setDipole(common::Vector3(0, 0, dipole_cmd.z()));
        
        // Step wheels
        for (auto& wheel : wheels) {
            wheel->step(dt);
        }
        
        // Print status every 30 seconds
        if (i % 30 == 0) {
            double h_total = desaturator.getTotalMomentum();
            std::string state_str;
            switch (desaturator.getState()) {
                case WheelDesaturation::State::IDLE: state_str = "IDLE"; break;
                case WheelDesaturation::State::DESATURATING: state_str = "DESATURATING"; break;
                case WheelDesaturation::State::COOLDOWN: state_str = "COOLDOWN"; break;
            }
            
            std::cout << std::setw(7) << (int)sim_time << " | "
                      << std::setw(13) << std::fixed << std::setprecision(4) << h_total << " | "
                      << std::setw(11) << std::fixed << std::setprecision(2) << (b_field.norm() * 1e6) << " | "
                      << std::setw(11) << std::scientific << std::setprecision(2) << dipole_cmd.norm() << " | "
                      << state_str << "\n";
        }
    }
    
    // === Results ===
    double h_total_final = 0.0;
    for (auto& wheel : wheels) {
        h_total_final += std::abs(wheel->getAngularMomentum());
    }
    
    std::cout << "\n=== Results ===\n";
    std::cout << "Initial momentum: " << std::fixed << std::setprecision(4) << h_total_saturated << " Nms\n";
    std::cout << "Final momentum:   " << std::fixed << std::setprecision(4) << h_total_final << " Nms\n";
    std::cout << "Reduction:        " << std::fixed << std::setprecision(1)
              << ((h_total_saturated - h_total_final) / h_total_saturated * 100.0) << "%\n";
    
    std::cout << "\nFinal state: ";
    switch (desaturator.getState()) {
        case WheelDesaturation::State::IDLE: std::cout << "IDLE\n"; break;
        case WheelDesaturation::State::DESATURATING: std::cout << "DESATURATING\n"; break;
        case WheelDesaturation::State::COOLDOWN: std::cout << "COOLDOWN\n"; break;
    }
    
    std::cout << "\n=== Demonstration Complete ===\n";
    
    return 0;
}
