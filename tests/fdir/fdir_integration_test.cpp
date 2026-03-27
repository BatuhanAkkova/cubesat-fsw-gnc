#include <gtest/gtest.h>

#include "fsw/core/ModeManager.hpp"
#include "fsw/fdir/FDIRManager.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"
#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimGyro.hpp"
#include "sim/models/SimMagnetometer.hpp"

using namespace fsw::fdir;
using namespace fsw::core;
using namespace fsw::gnc::ekf;
using namespace sim::dynamics;
using namespace sim::models;

/**
 * @brief Integration test demonstrating FDIR in a realistic scenario
 */
class FDIRIntegrationTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Setup rigid body dynamics
        common::Matrix3 inertia = common::Matrix3::Identity() * 0.1;
        body = std::make_unique<RigidBody>(inertia, common::Quaternion::Identity(), common::Vector3::Zero());

        // Setup orbit for magnetometer simulation
        common::Vector3 pos(6378137.0 + 400000.0, 0, 0);  // 400 km altitude
        common::Vector3 vel(0, 7670.0, 0);                // Circular velocity
        orbit = std::make_unique<Orbit>(pos, vel, Orbit::Config());

        // Setup FDIR manager
        fdir_manager = std::make_unique<FDIRManager>();

        // Setup mode manager
        mode_manager = std::make_unique<ModeManager>();

        // Setup MEKF
        mekf = std::make_unique<MEKF>();
        common::MatrixX P0(6, 6);
        P0.setIdentity();
        P0 *= 0.1;
        mekf->initialize(common::Quaternion::Identity(), common::Vector3::Zero(), P0);

        // Connect FDIR to ModeManager and MEKF
        fdir_manager->setModeManager(mode_manager.get());
        fdir_manager->setMEKF(mekf.get());

        // Register sensors
        fdir_manager->registerGyro("gyro");
        fdir_manager->registerMagnetometer("mag");
    }

    std::unique_ptr<RigidBody> body;
    std::unique_ptr<Orbit> orbit;
    std::unique_ptr<FDIRManager> fdir_manager;
    std::unique_ptr<ModeManager> mode_manager;
    std::unique_ptr<MEKF> mekf;
    double sim_time = 0.0;
};

/**
 * @brief Test FDIR with healthy sensors
 */
TEST_F(FDIRIntegrationTest, HealthySensorsNominalOperation) {
    SimGyro::Config gyro_cfg;
    gyro_cfg.noise_std = 1e-7;
    SimGyro gyro(*body, gyro_cfg);

    SimMagnetometer::Config mag_cfg;
    mag_cfg.noise_std = 1e-8;
    SimMagnetometer mag(*body, *orbit, mag_cfg);

    // Start in NOMINAL mode
    mode_manager->commandMode(MissionMode::NOMINAL);

    // Apply small external torque to create realistic sensor variations
    // (prevents false stuck detection on stationary satellite)
    common::Vector3 small_torque(1e-6, 5e-7, 3e-7);  // Very small torque [Nm]

    // Simulate for 10 seconds
    for (int i = 0; i < 100; i++) {
        // Read sensors
        common::Vector3 gyro_reading = gyro.read();
        common::Vector3 mag_reading = mag.read();

        // Update FDIR
        fdir_manager->updateSensor("gyro", gyro_reading, sim_time);
        fdir_manager->updateSensor("mag", mag_reading, sim_time);

        // Step simulation with small torque to create variation
        body->step(0.1, small_torque);
        orbit->step(0.1);
        sim_time += 0.1;
    }

    // Verify all sensors healthy
    EXPECT_EQ(fdir_manager->getSensorStatus("gyro"), HealthStatus::HEALTHY);
    EXPECT_EQ(fdir_manager->getSensorStatus("mag"), HealthStatus::HEALTHY);
    EXPECT_EQ(fdir_manager->getSystemHealth(), HealthStatus::HEALTHY);

    // Verify mode stayed NOMINAL
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::NOMINAL);
}

/**
 * @brief Test FDIR response to stuck gyro
 */
TEST_F(FDIRIntegrationTest, StuckGyroTriggersResponse) {
    // Start in NOMINAL mode
    mode_manager->commandMode(MissionMode::NOMINAL);
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::NOMINAL);

    // Feed stuck gyro readings
    common::Vector3 stuck_reading(0.1, 0.2, 0.3);

    for (int i = 0; i < 100; i++) {
        fdir_manager->updateSensor("gyro", stuck_reading, sim_time);
        fdir_manager->updateSensor("mag", common::Vector3(3e-5, 2e-5, 4e-5), sim_time);
        sim_time += 0.1;
    }

    // Verify gyro detected as FAILED
    EXPECT_EQ(fdir_manager->getSensorStatus("gyro"), HealthStatus::FAILED);

    // Verify forced transition to SAFE mode (gyro is critical)
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::SAFE);
}

/**
 * @brief Test FDIR response to out-of-range magnetometer
 */
TEST_F(FDIRIntegrationTest, OutOfRangeMagnetometer) {
    // Feed normal gyro, abnormal mag
    common::Vector3 normal_gyro(0.01, 0.02, 0.03);
    common::Vector3 abnormal_mag(1e-2, 0, 0);  // 10 mT >> Earth field

    for (int i = 0; i < 10; i++) {
        fdir_manager->updateSensor("gyro", normal_gyro, sim_time);
        fdir_manager->updateSensor("mag", abnormal_mag, sim_time);
        sim_time += 0.1;

        // Add small variation to prevent stuck detection
        normal_gyro.x() += 1e-5;
        abnormal_mag.x() += 1e-6;
    }

    // Verify mag is DEGRADED
    EXPECT_EQ(fdir_manager->getSensorStatus("mag"), HealthStatus::DEGRADED);

    // Gyro should still be healthy
    EXPECT_EQ(fdir_manager->getSensorStatus("gyro"), HealthStatus::HEALTHY);

    // System health should be degraded (not failed, since mag is not critical)
    EXPECT_EQ(fdir_manager->getSystemHealth(), HealthStatus::DEGRADED);
}

/**
 * @brief Test MEKF reset on critical sensor failure
 */
TEST_F(FDIRIntegrationTest, MEKFResetOnCriticalFailure) {
    // Enable auto MEKF reset
    fdir_manager->enableAutoMEKFReset(true);

    // Perturb MEKF state
    common::Quaternion perturbed_q(0.9, 0.1, 0.1, 0.1);
    perturbed_q.normalize();
    common::Vector3 bias(0.05, 0.05, 0.05);
    common::MatrixX P(6, 6);
    P.setIdentity();
    P *= 0.5;
    mekf->initialize(perturbed_q, bias, P);

    // Verify MEKF has non-identity state
    EXPECT_NE(mekf->getAttitude().w(), 1.0);
    EXPECT_GT(mekf->getBias().norm(), 0.0);

    // Trigger gyro failure (critical sensor)
    common::Vector3 stuck_gyro(0.1, 0.2, 0.3);
    for (int i = 0; i < 60; i++) {
        fdir_manager->updateSensor("gyro", stuck_gyro, sim_time);
        sim_time += 0.1;
    }

    // Verify MEKF was reset (should be back to identity attitude, zero bias)
    EXPECT_NEAR(mekf->getAttitude().w(), 1.0, 1e-6);
    EXPECT_NEAR(mekf->getBias().norm(), 0.0, 1e-6);
}

/**
 * @brief Test health summary reporting
 */
TEST_F(FDIRIntegrationTest, HealthSummaryReporting) {
    // All sensors start healthy
    auto summary = fdir_manager->getHealthSummary();
    EXPECT_EQ(summary.healthy_count, 2);
    EXPECT_EQ(summary.degraded_count, 0);
    EXPECT_EQ(summary.failed_count, 0);

    // Make mag degraded
    common::Vector3 bad_mag(1e-2, 0, 0);
    fdir_manager->updateSensor("mag", bad_mag, sim_time);

    summary = fdir_manager->getHealthSummary();
    EXPECT_EQ(summary.healthy_count, 1);
    EXPECT_EQ(summary.degraded_count, 1);
    EXPECT_EQ(summary.failed_count, 0);

    // Make gyro failed
    common::Vector3 stuck_gyro(0.1, 0.2, 0.3);
    for (int i = 0; i < 60; i++) {
        fdir_manager->updateSensor("gyro", stuck_gyro, sim_time);
        sim_time += 0.1;
    }

    summary = fdir_manager->getHealthSummary();
    EXPECT_EQ(summary.healthy_count, 0);
    EXPECT_EQ(summary.degraded_count, 1);
    EXPECT_EQ(summary.failed_count, 1);
}

/**
 * @brief Test reset all functionality
 */
TEST_F(FDIRIntegrationTest, ResetAllMonitors) {
    // Trigger faults
    common::Vector3 stuck_gyro(0.1, 0.2, 0.3);
    common::Vector3 bad_mag(1e-2, 0, 0);

    for (int i = 0; i < 60; i++) {
        fdir_manager->updateSensor("gyro", stuck_gyro, sim_time);
        fdir_manager->updateSensor("mag", bad_mag, sim_time);
        sim_time += 0.1;
    }

    // Verify faults
    EXPECT_NE(fdir_manager->getSensorStatus("gyro"), HealthStatus::HEALTHY);
    EXPECT_NE(fdir_manager->getSensorStatus("mag"), HealthStatus::HEALTHY);

    // Reset all
    fdir_manager->resetAll();

    // Verify all healthy
    EXPECT_EQ(fdir_manager->getSensorStatus("gyro"), HealthStatus::HEALTHY);
    EXPECT_EQ(fdir_manager->getSensorStatus("mag"), HealthStatus::HEALTHY);
}

/**
 * @brief Test sensor redundancy switching
 */
TEST_F(FDIRIntegrationTest, PrimaryFailureSwitchesToBackup) {
    // Register primary and backup gyros
    fdir_manager->registerGyro("gyro_primary");
    fdir_manager->registerGyro("gyro_backup");
    fdir_manager->registerRedundantPair("gyro_group", "gyro_primary", "gyro_backup");

    // Start in NOMINAL mode
    mode_manager->commandMode(MissionMode::NOMINAL);
    EXPECT_EQ(fdir_manager->getActiveSensor("gyro_group"), "gyro_primary");

    // Trigger primary gyro failure
    common::Vector3 stuck_reading(0.1, 0.2, 0.3);
    for (int i = 0; i < 70; i++) {
        fdir_manager->updateSensor("gyro_primary", stuck_reading, sim_time);
        sim_time += 0.1;
    }

    // Verify primary is FAILED
    EXPECT_EQ(fdir_manager->getSensorStatus("gyro_primary"), HealthStatus::FAILED);

    // Verify switch to backup
    EXPECT_EQ(fdir_manager->getActiveSensor("gyro_group"), "gyro_backup");

    // Verify transition to DEGRADED mode instead of SAFE
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::DEGRADED);
}

/**
 * @brief Test degraded sensor triggering DEGRADED mode
 */
TEST_F(FDIRIntegrationTest, DegradedSensorTriggersDegradedMode) {
    // Start in NOMINAL mode
    mode_manager->commandMode(MissionMode::NOMINAL);

    // Feed abnormal but not failed mag readings
    common::Vector3 abnormal_mag(1e-2, 0, 0);  // 10 mT

    for (int i = 0; i < 10; i++) {
        fdir_manager->updateSensor("gyro", common::Vector3(0.01, 0.02, 0.03), sim_time);
        fdir_manager->updateSensor("mag", abnormal_mag, sim_time);
        sim_time += 0.1;
        abnormal_mag.x() += 1e-6;  // prevent stuck
    }

    // Verify mag is DEGRADED
    EXPECT_EQ(fdir_manager->getSensorStatus("mag"), HealthStatus::DEGRADED);

    // Verify system moved to DEGRADED mode
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::DEGRADED);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
