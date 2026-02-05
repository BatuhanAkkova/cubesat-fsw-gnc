#include <gtest/gtest.h>
#include "sim/dynamics/Orbit.hpp"
#include <cmath>
#include <chrono>
#include <iostream>

using namespace sim::dynamics;

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double MU_EARTH = 3.986004418e14;
constexpr double R_EARTH = 6378137.0;

TEST(OrbitJ2Test, J2DisabledMatchesKeplerian) {
    double altitude = 400000.0;
    double r = R_EARTH + altitude;
    double v_circ = std::sqrt(MU_EARTH / r);
    
    common::Vector3 init_pos(r, 0, 0);
    common::Vector3 init_vel(0, v_circ, 0);
    
    Orbit::Config config_j2_on;
    config_j2_on.enable_j2 = true;
    
    Orbit::Config config_j2_off;
    config_j2_off.enable_j2 = false;
    
    Orbit orbit_j2(init_pos, init_vel, config_j2_on);
    Orbit orbit_keplerian(init_pos, init_vel, config_j2_off);
    
    double dt = 0.1;
    for (int i = 0; i < 100; i++) {
        orbit_j2.step(dt);
        orbit_keplerian.step(dt);
    }
    
    common::Vector3 pos_j2 = orbit_j2.getPosition();
    common::Vector3 pos_kep = orbit_keplerian.getPosition();
    
    double diff = (pos_j2 - pos_kep).norm();
    EXPECT_GT(diff, 0.1);
    EXPECT_LT(diff, 100.0);
}

TEST(OrbitJ2Test, CircularOrbitElementsWithJ2) {
    double altitude = 400000.0;
    double r = R_EARTH + altitude;
    double v_circ = std::sqrt(MU_EARTH / r);
    double inclination = 51.6 * DEG_TO_RAD;
    
    common::Vector3 init_pos(r, 0, 0);
    common::Vector3 init_vel(0, v_circ * std::cos(inclination), v_circ * std::sin(inclination));
    
    Orbit::Config config;
    config.enable_j2 = true;
   
    Orbit orbit(init_pos, init_vel, config);
    
    double a0 = orbit.getSemiMajorAxis();
    double e0 = orbit.getEccentricity();
    double inc0 = orbit.getInclination();
    double raan0 = orbit.getRaan();
    
    double orbital_period = 2.0 * PI * std::sqrt(std::pow(r, 3) / MU_EARTH);
    
    double dt = 1.0;
    int num_steps = static_cast<int>(orbital_period / dt);
    
    for (int i = 0; i < num_steps; i++) {
        orbit.step(dt);
    }
    
    double a1 = orbit.getSemiMajorAxis();
    double e1 = orbit.getEccentricity();
    double inc1 = orbit.getInclination();
    double raan1 = orbit.getRaan();
    
    EXPECT_NEAR(a1, a0, 100.0);
    EXPECT_LT(e1, 0.01);
    EXPECT_NEAR(inc1, inc0, 0.001);
    
    double raan_change = raan1 - raan0;
    while (raan_change > PI) raan_change -= 2.0 * PI;
    while (raan_change < -PI) raan_change += 2.0 * PI;
    
    EXPECT_LT(std::abs(raan_change), 1.0 * DEG_TO_RAD);
    EXPECT_GT(std::abs(raan_change), 0.0001);
}

/**
 * @brief Performance test: J2 should add <10% overhead
 */
TEST(OrbitJ2Test, J2PerformanceOverhead) {
    double altitude = 400000.0;
    double r = R_EARTH + altitude;
    double v_circ = std::sqrt(MU_EARTH / r);
    
    common::Vector3 init_pos(r, 0, 0);
    common::Vector3 init_vel(0, v_circ, 0);
    
    // Test Keplerian performance
    Orbit::Config config_kep;
    config_kep.enable_j2 = false;
    Orbit orbit_kep(init_pos, init_vel, config_kep);
    
    auto start_kep = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        orbit_kep.step(0.1);
    }
    auto end_kep = std::chrono::high_resolution_clock::now();
    auto duration_kep = std::chrono::duration_cast<std::chrono::microseconds>(end_kep - start_kep);
    
    // Test J2 performance
    Orbit::Config config_j2;
    config_j2.enable_j2 = true;
    Orbit orbit_j2(init_pos, init_vel, config_j2);
    
    auto start_j2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        orbit_j2.step(0.1);
    }
    auto end_j2 = std::chrono::high_resolution_clock::now();
    auto duration_j2 = std::chrono::duration_cast<std::chrono::microseconds>(end_j2 - start_j2);
    
    // J2 should add less than 10% overhead
    double overhead = static_cast<double>(duration_j2.count()) / duration_kep.count();
    
    std::cout << "Keplerian time: " << duration_kep.count() << " μs\n";
    std::cout << "J2 time: " << duration_j2.count() << " μs\n";
    std::cout << "Overhead: " << (overhead - 1.0) * 100.0 << "%\n";
    
    EXPECT_LT(overhead, 1.10);  // Less than 10% overhead
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
