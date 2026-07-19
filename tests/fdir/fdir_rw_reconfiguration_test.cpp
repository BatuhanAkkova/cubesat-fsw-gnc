#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "fsw/core/ModeManager.hpp"
#include "fsw/fdir/FDIRManager.hpp"
#include "fsw/gnc/control/ControlAllocator.hpp"

using namespace fsw::fdir;
using namespace fsw::core;
using namespace fsw::gnc::control;

TEST(FDIRRWReconfigurationTest, NominalAllocation) {
    ControlAllocator allocator;
    common::Vector3 cmd_torque(0.01, -0.005, 0.002);
    std::vector<double> allocated_torques;

    bool success = allocator.allocate(cmd_torque, allocated_torques);
    EXPECT_TRUE(success);
    EXPECT_EQ(allocated_torques.size(), 4);

    // Reconstruct body torque from allocated wheel torques:
    // Torque_body = - A * u
    double c = 0.7071067811865475;
    double s = 0.7071067811865475;
    Eigen::MatrixXd A(3, 4);
    A <<  c,  0.0, -c,  0.0,
         0.0,  c,  0.0, -c,
          s,   s,   s,   s;

    Eigen::VectorXd u(4);
    for (int i = 0; i < 4; ++i) {
        u(i) = allocated_torques[i];
    }

    Eigen::Vector3d recon = - A * u;
    EXPECT_NEAR(recon(0), cmd_torque.x(), 1e-6);
    EXPECT_NEAR(recon(1), cmd_torque.y(), 1e-6);
    EXPECT_NEAR(recon(2), cmd_torque.z(), 1e-6);
}

TEST(FDIRRWReconfigurationTest, ReconfigurationSingleWheelFailure) {
    ControlAllocator allocator;
    
    // Simulate Wheel 0 (index 0) failure
    allocator.setWheelHealth(0, false);
    EXPECT_FALSE(allocator.getWheelHealth(0));
    EXPECT_TRUE(allocator.getWheelHealth(1));

    common::Vector3 cmd_torque(0.01, -0.005, 0.002);
    std::vector<double> allocated_torques;

    bool success = allocator.allocate(cmd_torque, allocated_torques);
    EXPECT_TRUE(success);
    EXPECT_NEAR(allocated_torques[0], 0.0, 1e-9); // Wheel 0 should not be commanded

    // Reconstruct body torque from active wheels only
    double c = 0.7071067811865475;
    double s = 0.7071067811865475;
    Eigen::MatrixXd A(3, 4);
    A << 0.0,  0.0, -c,  0.0, // Column 0 is zeroed out
         0.0,  c,  0.0, -c,
         0.0,   s,   s,   s;

    Eigen::VectorXd u(4);
    for (int i = 0; i < 4; ++i) {
        u(i) = allocated_torques[i];
    }

    Eigen::Vector3d recon = - A * u;
    EXPECT_NEAR(recon(0), cmd_torque.x(), 1e-6);
    EXPECT_NEAR(recon(1), cmd_torque.y(), 1e-6);
    EXPECT_NEAR(recon(2), cmd_torque.z(), 1e-6);
}

TEST(FDIRRWReconfigurationTest, FDIRWheelDiagnosticsAndTransitions) {
    FDIRManager fdir;
    ModeManager mode_manager;
    fdir.setModeManager(&mode_manager);
    
    // Initialize state
    mode_manager.forceModeChange(MissionMode::NOMINAL, "Test Start");
    EXPECT_EQ(mode_manager.getCurrentMode(), MissionMode::NOMINAL);

    std::vector<double> speeds = {0.0, 0.0, 0.0, 0.0};
    std::vector<double> commands = {0.01, 0.0, 0.0, 0.0}; // Command Wheel 0
    double dt = 0.1;

    // Wheel 0 speed is stuck at 0.0 despite commands
    for (int i = 0; i < 15; ++i) {
        fdir.updateWheels(speeds, commands, dt, i * dt);
    }

    // FDIR should detect Wheel 0 as stuck/failed
    EXPECT_EQ(fdir.getWheelStatus(0), HealthStatus::FAILED);
    EXPECT_EQ(fdir.getWheelStatus(1), HealthStatus::HEALTHY);
    EXPECT_EQ(fdir.getFailedWheelCount(), 1);

    // Should transition to DEGRADED mode
    EXPECT_EQ(mode_manager.getCurrentMode(), MissionMode::DEGRADED);

    // Fail a second wheel (Wheel 1)
    commands = {0.0, 0.01, 0.0, 0.0};
    for (int i = 15; i < 30; ++i) {
        fdir.updateWheels(speeds, commands, dt, i * dt);
    }

    EXPECT_EQ(fdir.getWheelStatus(1), HealthStatus::FAILED);
    EXPECT_EQ(fdir.getFailedWheelCount(), 2);

    // Multiple wheel failures -> SAFE mode transition
    EXPECT_EQ(mode_manager.getCurrentMode(), MissionMode::SAFE);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
