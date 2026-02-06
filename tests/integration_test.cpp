#include <gtest/gtest.h>
#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimMagnetometer.hpp"
#include "sim/models/SimRW.hpp"
#include "fsw/gnc/control/WheelDesaturation.hpp"
#include <memory>

using namespace sim;
using namespace sim::dynamics;
using namespace sim::models;
using namespace fsw::gnc::control;

/**
 * @brief Integration test for J2 orbit propagator, time-varying B-field, and wheel desaturation together.
 */
TEST(IntegrationTest, AllComponentsTogether) {
    // === Setup Orbit with J2 ===
    double altitude = 400000.0;  // 400 km LEO
    double r = 6378137.0 + altitude;
    double v_circ = std::sqrt(3.986004418e14 / r);
    double inclination = 51.6 * 3.14159265358979323846 / 180.0;
    
    common::Vector3 init_pos(r, 0, 0);
    common::Vector3 init_vel(0, v_circ * std::cos(inclination), v_circ * std::sin(inclination));
    
    Orbit::Config orbit_config;
    orbit_config.enable_j2 = true;
    Orbit orbit(init_pos, init_vel, orbit_config);
    
    // === Setup Rigid Body ===
    common::Matrix3 inertia = common::Matrix3::Identity() * 0.1;  // 0.1 kg*m^2
    common::Quaternion init_attitude = common::Quaternion::Identity();
    common::Vector3 init_omega = common::Vector3::Zero();
    RigidBody body(inertia, init_attitude, init_omega);
    
    // === Setup Time-Varying Magnetometer ===
    SimMagnetometer::Config mag_config;
    mag_config.enable_earth_rotation = true;
    mag_config.use_tilted_dipole = false;
    SimMagnetometer magnetometer(body, orbit, mag_config);
    
    // === Setup Reaction Wheels ===
    std::vector<std::shared_ptr<SimRW>> wheels;
    SimRW::Config rw_config;
    rw_config.inertia = 0.001;  // 0.001 kg*m^2
    rw_config.max_torque = 0.01;  // 0.01 Nm
    rw_config.max_momentum = 0.05;  // 0.05 Nms
    rw_config.friction_coeff = 0.0;
    rw_config.initial_speed = 0.0;
    
    for (int i = 0; i < 3; i++) {
        wheels.push_back(std::make_shared<SimRW>(rw_config));
    }
    
    // Saturate wheels to 90% capacity
    wheels[0]->setTorqueCommand(0.01);
    wheels[1]->setTorqueCommand(0.01);
    wheels[2]->setTorqueCommand(0.01);
    for (int i = 0; i < 5000; i++) {
        wheels[0]->step(0.1);
        wheels[1]->step(0.1);
        wheels[2]->step(0.1);
    }
    
    // Convert to IRW interface for desaturator
    std::vector<std::shared_ptr<hal::IRW>> wheels_interface(wheels.begin(), wheels.end());
    
    // === Setup Wheel Desaturation ===
    WheelDesaturation::Config desat_config;
    desat_config.momentum_threshold_ratio = 0.8;
    desat_config.desat_gain = 1e-6;
    WheelDesaturation desaturator(desat_config);
    
    // === Integration Test Loop ===
    double sim_time = 0.0;
    double dt = 1.0;  // 1 second timestep
    int num_steps = 100;  // 100 seconds
    
    bool desaturation_triggered = false;
    double initial_momentum = 0.0;
    double final_momentum = 0.0;
    
    for (int step = 0; step < num_steps; step++) {
        // Update simulation time
        sim_time += dt;
        magnetometer.setTime(sim_time);
        
        // Propagate orbit with J2
        orbit.step(dt);
        
        // Read B-field (should vary with Earth rotation)
        common::Vector3 b_field = magnetometer.read();
        
        // Update wheel desaturation
        common::Vector3 dipole_cmd = desaturator.update(wheels_interface, b_field, dt);
        
        // Check if desaturation triggered
        if (desaturator.getState() == WheelDesaturation::State::DESATURATING) {
            desaturation_triggered = true;
        }
        
        // Track momentum
        if (step == 0) {
            initial_momentum = desaturator.getTotalMomentum();
        }
        if (step == num_steps - 1) {
            final_momentum = desaturator.getTotalMomentum();
        }
    }
    
    // === Verification ===
    
    // 1. Orbit should have propagated
    common::Vector3 final_pos = orbit.getPosition();
    EXPECT_GT(final_pos.norm(), 0.0);
    EXPECT_NE(final_pos, init_pos);  // Position should have changed
    
    // 2. Orbital elements should be reasonable
    double semi_major_axis = orbit.getSemiMajorAxis();
    EXPECT_NEAR(semi_major_axis, r, 1000.0);  // Within 1km
    
    double eccentricity = orbit.getEccentricity();
    EXPECT_LT(eccentricity, 0.01);  // Nearly circular
    
    // 3. Desaturation should have triggered
    EXPECT_TRUE(desaturation_triggered) << "Desaturation should trigger with saturated wheels";
    
    // 4. Initial momentum should be high
    EXPECT_GT(initial_momentum, 0.04);  // >80% of 0.05 Nms max
    
    std::cout << "Integration Test Results:\n";
    std::cout << "  Initial momentum: " << initial_momentum << " Nms\n";
    std::cout << "  Final momentum: " << final_momentum << " Nms\n";
    std::cout << "  Semi-major axis: " << semi_major_axis / 1000.0 << " km\n";
    std::cout << "  Eccentricity: " << eccentricity << "\n";
    std::cout << "  Desaturation triggered: " << (desaturation_triggered ? "YES" : "NO") << "\n";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
