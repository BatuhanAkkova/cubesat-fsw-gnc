#include <gtest/gtest.h>
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimTorquer.hpp"
#include "fsw/gnc/control/Bdot.hpp"
#include <iostream>

using namespace common;

class Phase3BdotTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Inertia matrix (arbitrary diagonal)
        Matrix3 inertia;
        inertia << 0.01, 0.0, 0.0,
                   0.0, 0.01, 0.0,
                   0.0, 0.0, 0.01;

        // Initial state: Tumble
        Quaternion q_init(1, 0, 0, 0); // Identity
        Vector3 w_init(0.1, 0.1, 0.1); // approx 5.7 deg/sec per axis

        body = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);
        
        // Max dipole 1.0 Am^2
        torquer = std::make_unique<sim::models::SimTorquer>(1.0);

        // High gain
        bdot = std::make_unique<fsw::gnc::control::Bdot>(100000.0); 
    }

    std::unique_ptr<sim::dynamics::RigidBody> body;
    std::unique_ptr<sim::models::SimTorquer> torquer;
    std::unique_ptr<fsw::gnc::control::Bdot> bdot;

    const Vector3 B_inertial = {0.0, 50000e-9, 0.0}; // 50 uT in Y
};

TEST_F(Phase3BdotTest, DetumbleSimulation) {
    double dt = 0.1;
    double simulation_time = 60.0; // seconds
    int steps = static_cast<int>(simulation_time / dt);

    double initial_kinetic_energy = 0.5 * body->getAngularVelocity().dot(body->getAngularVelocity()); // Approx (if inertia is diagonal scalar scaled)
    // Actually E = 0.5 * w^T * I * w
    // Since I is 0.1 * Identity, E = 0.5 * 0.1 * |w|^2
    
    std::cout << "Initial Angular Velocity: " << body->getAngularVelocity().transpose() << std::endl;

    for (int i = 0; i < steps; ++i) {
        // 1. Get Environment
        Quaternion q_bi = body->getAttitude();
        Vector3 B_body = q_bi.conjugate() * B_inertial;

        // 2. Control
        Vector3 dipole_cmd = bdot->update(B_body, dt);

        // 3. Actuate
        torquer->setDipole(dipole_cmd);
        Vector3 M_actual = torquer->getDipole();

        // 4. Dynamics impact
        // Torque = M x B
        Vector3 torque_body = M_actual.cross(B_body);
        
        // 5. Propagate
        body->step(dt, torque_body);
    }

    Vector3 w_final = body->getAngularVelocity();
    std::cout << "Final Angular Velocity: " << w_final.transpose() << std::endl;

    // Check magnitude
    // Note: With constant B-field, B-dot cannot fully detumble the axis aligned with B.
    // However, it should significantly reduce kinetic energy.
    EXPECT_LT(w_final.norm(), 0.12) << "Spacecraft should have significantly detumbled";
    
    // Check that we slowed down from initial ~0.173
    EXPECT_LT(w_final.norm(), 0.17);
}
