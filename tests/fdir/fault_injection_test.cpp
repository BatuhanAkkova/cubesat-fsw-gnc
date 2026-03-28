#include <gtest/gtest.h>

#include "fsw/core/ModeManager.hpp"
#include "fsw/fdir/FDIRConfig.hpp"
#include "fsw/fdir/FDIRManager.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"

#include "sim/Simulation.hpp"

using namespace fsw::fdir;
using namespace fsw::core;
using namespace fsw::gnc::ekf;
using namespace common;

class FaultInjectionTest : public ::testing::Test {
   protected:
    void SetUp() override {
        fdir = std::make_unique<FDIRManager>();
        mode_manager = std::make_unique<ModeManager>();
        mekf = std::make_unique<MEKF>();

        fdir->setModeManager(mode_manager.get());
        fdir->setMEKF(mekf.get());

        // Ensure NOMINAL mode for testing failover from nominal
        mode_manager->forceModeChange(MissionMode::NOMINAL, "Initial Test State");

        // Register redundant gyros
        GyroHealthConfig gyro_cfg;
        gyro_cfg.stuck_sample_count = 3;
        gyro_cfg.stuck_tolerance = 1e-6;

        fdir->registerGyro("gyro_primary", gyro_cfg);
        fdir->registerGyro("gyro_backup", gyro_cfg);
        fdir->registerRedundantPair("gyro", "gyro_primary", "gyro_backup");

        // Register magnetometer (critical)
        MagnetometerHealthConfig mag_cfg;
        fdir->registerMagnetometer("mag", mag_cfg);
    }

    std::unique_ptr<FDIRManager> fdir;
    std::unique_ptr<ModeManager> mode_manager;
    std::unique_ptr<MEKF> mekf;
    double t = 0.0;
};

TEST_F(FaultInjectionTest, GyroFailoverToBackup) {
    EXPECT_EQ(fdir->getActiveSensor("gyro"), "gyro_primary");
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::NOMINAL);

    // Step 1: Normal readings
    fdir->updateSensor("gyro_primary", Vector3(0.1, 0.0, 0.0), t);
    t += 0.1;
    fdir->updateSensor("gyro_primary", Vector3(0.11, 0.0, 0.0), t);
    t += 0.1;

    EXPECT_EQ(fdir->getSensorStatus("gyro_primary"), HealthStatus::HEALTHY);

    // Step 2: Inject "Stuck" fault on primary
    Vector3 stuck_reading(0.11, 0.0, 0.0);
    for (int i = 0; i < 5; ++i) {
        fdir->updateSensor("gyro_primary", stuck_reading, t);
        t += 0.1;
    }

    // Should have failed over to backup
    EXPECT_EQ(fdir->getSensorStatus("gyro_primary"), HealthStatus::FAILED);
    EXPECT_EQ(fdir->getActiveSensor("gyro"), "gyro_backup");

    // Failover triggers DEGRADED mode in our implementation
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::DEGRADED);
}

TEST_F(FaultInjectionTest, DoubleGyroFailureToSafeMode) {
    // 1. Fail primary
    Vector3 stuck(0.1, 0.0, 0.0);
    for (int i = 0; i < 10; ++i) fdir->updateSensor("gyro_primary", stuck, t);

    EXPECT_EQ(fdir->getActiveSensor("gyro"), "gyro_backup");

    // 2. Fail backup
    for (int i = 0; i < 10; ++i) fdir->updateSensor("gyro_backup", stuck, t);

    EXPECT_EQ(fdir->getSensorStatus("gyro_backup"), HealthStatus::FAILED);

    // No more backups -> SAFE mode
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::SAFE);
}

TEST_F(FaultInjectionTest, MagSpikeToDegradedMode) {
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::NOMINAL);

    // Inject spike (Out of Range)
    fdir->updateSensor("mag", Vector3(0.1, 0.1, 0.1), t);  // High field

    EXPECT_EQ(fdir->getSensorStatus("mag"), HealthStatus::DEGRADED);
    EXPECT_EQ(mode_manager->getCurrentMode(), MissionMode::DEGRADED);
}
