#include <gtest/gtest.h>
#include "fsw/core/ModeManager.hpp"
#include <iostream>

using namespace fsw::core;
using namespace common;

class ModeManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        config.safe_to_nominal_rate_threshold = 0.02;
        config.nominal_to_safe_rate_threshold = 0.1;
        config.min_time_in_mode = 5.0;  // Shorter for testing
    }

    ModeTransitionConfig config;
};

TEST_F(ModeManagerTest, InitialMode) {
    ModeManager manager(config);
    
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
    EXPECT_EQ(manager.getTimeInMode(), 0.0);
}

TEST_F(ModeManagerTest, SafeToNominalTransition) {
    ModeManager manager(config);
    
    // Start in SAFE mode with high rates
    Vector3 high_rate(0.15, 0.1, 0.05);  // Above threshold
    
    // Update for some time, should stay in SAFE
    for (int i = 0; i < 30; ++i) {
        manager.update(high_rate, 0.1);
    }
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
    
    // Now reduce rates below threshold
    Vector3 low_rate(0.01, 0.005, 0.008);  // Below 0.02 threshold
    
    // Update for enough time to allow transition
    for (int i = 0; i < 60; ++i) {
        manager.update(low_rate, 0.1);
    }
    
    // Should have transitioned to NOMINAL
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
}

TEST_F(ModeManagerTest, NominalToSafeTransition) {
    ModeManager manager(config);
    
    // Force to NOMINAL mode
    manager.commandMode(MissionMode::NOMINAL);
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
    
    // Stay with low rates
    Vector3 low_rate(0.01, 0.005, 0.008);
    for (int i = 0; i < 30; ++i) {
        manager.update(low_rate, 0.1);
    }
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
    
    // Suddenly high rates (disturbance)
    Vector3 high_rate(0.15, 0.1, 0.05);
    
    // Update for enough time to allow transition
    for (int i = 0; i < 60; ++i) {
        manager.update(high_rate, 0.1);
    }
    
    // Should have returned to SAFE
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
}

TEST_F(ModeManagerTest, MinTimeInModeEnforced) {
    ModeManager manager(config);
    
    // Start in SAFE with low rates (would normally trigger transition)
    Vector3 low_rate(0.01, 0.005, 0.008);
    
    // Update for less than min_time_in_mode
    for (int i = 0; i < 40; ++i) {  // 4 seconds
        manager.update(low_rate, 0.1);
    }
    
    // Should NOT have transitioned yet
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
    EXPECT_LT(manager.getTimeInMode(), config.min_time_in_mode);
    
    // Continue updating past threshold
    for (int i = 0; i < 20; ++i) {  // Additional 2 seconds -> total 6s
        manager.update(low_rate, 0.1);
    }
    
    // NOW should have transitioned
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
}

TEST_F(ModeManagerTest, ModeChangeCallback) {
    ModeManager manager(config);
    
    bool callback_invoked = false;
    MissionMode from_mode = MissionMode::SAFE;
    MissionMode to_mode = MissionMode::SAFE;
    
    manager.setModeChangeCallback([&](MissionMode from, MissionMode to) {
        callback_invoked = true;
        from_mode = from;
        to_mode = to;
    });
    
    // Command a mode change
    manager.commandMode(MissionMode::NOMINAL);
    
    EXPECT_TRUE(callback_invoked);
    EXPECT_EQ(from_mode, MissionMode::SAFE);
    EXPECT_EQ(to_mode, MissionMode::NOMINAL);
}

TEST_F(ModeManagerTest, ManualModeCommand) {
    ModeManager manager(config);
    
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
    
    // Manual command to NOMINAL
    bool success = manager.commandMode(MissionMode::NOMINAL);
    EXPECT_TRUE(success);
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
    
    // Command same mode (should be OK)
    success = manager.commandMode(MissionMode::NOMINAL);
    EXPECT_TRUE(success);
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
}

TEST_F(ModeManagerTest, Reset) {
    ModeManager manager(config);
    
    // Go to NOMINAL
    manager.commandMode(MissionMode::NOMINAL);
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::NOMINAL);
    
    // Reset should return to SAFE
    manager.reset();
    EXPECT_EQ(manager.getCurrentMode(), MissionMode::SAFE);
    EXPECT_EQ(manager.getTimeInMode(), 0.0);
}

TEST_F(ModeManagerTest, GetModeString) {
    EXPECT_EQ(ModeManager::getModeString(MissionMode::SAFE), "SAFE");
    EXPECT_EQ(ModeManager::getModeString(MissionMode::NOMINAL), "NOMINAL");
    EXPECT_EQ(ModeManager::getModeString(MissionMode::CONTINGENCY), "CONTINGENCY");
}
