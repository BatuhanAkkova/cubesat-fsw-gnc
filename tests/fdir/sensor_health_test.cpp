#include <gtest/gtest.h>

#include "fsw/fdir/FDIRConfig.hpp"
#include "fsw/fdir/SensorHealthMonitor.hpp"

using namespace fsw::fdir;

/**
 * @brief Test fixture for SensorHealthMonitor tests
 */
class SensorHealthMonitorTest : public ::testing::Test {
   protected:
    void SetUp() override {
        gyro_monitor = std::make_unique<SensorHealthMonitor<common::Vector3>>("test_gyro");

        // Configure for quick detection in tests
        GyroHealthConfig config;
        config.stuck_tolerance = 1e-6;
        config.stuck_sample_count = 3;  // Detect quick for testing
        config.max_rate = 5.0;          // rad/s
        config.max_rate_change = 1.0;   // rad/s^2
        config.dt = 0.1;                // 10 Hz

        gyro_monitor->setStuckThreshold(config.stuck_tolerance, config.stuck_sample_count);
        gyro_monitor->setRangeThresholds(0.0, config.max_rate);
        gyro_monitor->setRapidChangeThreshold(config.max_rate_change, config.dt);
    }

    std::unique_ptr<SensorHealthMonitor<common::Vector3>> gyro_monitor;
    double test_time = 0.0;
};

/**
 * @brief Test stuck sensor detection
 */
TEST_F(SensorHealthMonitorTest, DetectStuckSensor) {
    common::Vector3 reading(0.1, 0.2, 0.3);

    // Feed same reading multiple times
    for (int i = 0; i < 5; i++) {
        gyro_monitor->update(reading, test_time);
        test_time += 0.1;
    }

    // Should detect as FAILED after stuck_sample_count
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::FAILED);
}

/**
 * @brief Test that small changes prevent stuck detection
 */
TEST_F(SensorHealthMonitorTest, SmallChangesPreventStuck) {
    common::Vector3 reading(0.1, 0.2, 0.3);

    // Feed readings with small variations
    for (int i = 0; i < 10; i++) {
        reading.x() += 1e-5;  // Small change above tolerance
        gyro_monitor->update(reading, test_time);
        test_time += 0.1;
    }

    // Should remain HEALTHY
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::HEALTHY);
}

/**
 * @brief Test out-of-range detection
 */
TEST_F(SensorHealthMonitorTest, DetectOutOfRange) {
    // Feed reading above max_rate
    common::Vector3 reading(10.0, 0.0, 0.0);  // 10 rad/s > 5 rad/s limit

    gyro_monitor->update(reading, test_time);

    // Should detect as DEGRADED immediately
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::DEGRADED);
}

/**
 * @brief Test rapid change detection
 */
TEST_F(SensorHealthMonitorTest, DetectRapidChange) {
    // Start with zero
    common::Vector3 reading1(0.0, 0.0, 0.0);
    gyro_monitor->update(reading1, test_time);
    test_time += 0.1;

    // Jump to large value (exceeds max_change_per_sample)
    // max_change_per_sample = 1.0 * 0.1 = 0.1 rad/s
    common::Vector3 reading2(1.0, 0.0, 0.0);  // Change of 1.0 > 0.1
    gyro_monitor->update(reading2, test_time);

    // Should detect as DEGRADED
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::DEGRADED);
}

/**
 * @brief Test that normal operation stays healthy
 */
TEST_F(SensorHealthMonitorTest, NormalOperationHealthy) {
    // Simulate realistic gyro readings
    double omega = 0.1;  // rad/s

    for (int i = 0; i < 100; i++) {
        common::Vector3 reading(omega, omega * 0.5, omega * 0.3);
        gyro_monitor->update(reading, test_time);
        test_time += 0.1;

        // Add small noise
        omega += (rand() % 100 - 50) * 1e-5;
    }

    // Should remain HEALTHY
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::HEALTHY);
}

/**
 * @brief Test status change callback
 */
TEST_F(SensorHealthMonitorTest, StatusChangeCallback) {
    bool callback_triggered = false;
    HealthStatus reported_status = HealthStatus::HEALTHY;
    std::string reported_message;

    gyro_monitor->setStatusChangeCallback([&](const std::string& name, HealthStatus status, const std::string& msg) {
        callback_triggered = true;
        reported_status = status;
        reported_message = msg;
    });

    // Trigger stuck detection
    common::Vector3 reading(0.1, 0.2, 0.3);
    for (int i = 0; i < 5; i++) {
        gyro_monitor->update(reading, test_time);
        test_time += 0.1;
    }

    EXPECT_TRUE(callback_triggered);
    EXPECT_EQ(reported_status, HealthStatus::FAILED);
    EXPECT_FALSE(reported_message.empty());
}

/**
 * @brief Test reset functionality
 */
TEST_F(SensorHealthMonitorTest, ResetMonitor) {
    // Trigger fault
    common::Vector3 reading(10.0, 0.0, 0.0);  // Out of range
    gyro_monitor->update(reading, test_time);
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::DEGRADED);

    // Reset
    gyro_monitor->reset();
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::HEALTHY);

    // Verify normal operation after reset
    common::Vector3 normal_reading(0.5, 0.0, 0.0);
    gyro_monitor->update(normal_reading, test_time);
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::HEALTHY);
}

/**
 * @brief Test enabling/disabling detection strategies
 */
TEST_F(SensorHealthMonitorTest, DisableDetectionStrategies) {
    // Disable stuck detection
    gyro_monitor->enableStuckDetection(false);

    // Feed stuck data
    common::Vector3 reading(0.1, 0.2, 0.3);
    for (int i = 0; i < 10; i++) {
        gyro_monitor->update(reading, test_time);
        test_time += 0.1;
    }

    // Should NOT detect as failed since stuck detection is disabled
    EXPECT_EQ(gyro_monitor->getStatus(), HealthStatus::HEALTHY);
}

/**
 * @brief Test magnetometer-specific configuration
 */
TEST(MagnetometerHealthTest, Configuration) {
    SensorHealthMonitor<common::Vector3> mag_monitor("test_mag");

    MagnetometerHealthConfig config;
    mag_monitor.setStuckThreshold(config.stuck_tolerance, config.stuck_sample_count);
    mag_monitor.setRangeThresholds(config.min_field, config.max_field);
    mag_monitor.setRapidChangeThreshold(config.max_field_change, config.dt);

    // Test typical Earth magnetic field (should be healthy)
    common::Vector3 earth_field(3e-5, 2e-5, 4e-5);  // ~50 µT
    mag_monitor.update(earth_field, 0.0);

    EXPECT_EQ(mag_monitor.getStatus(), HealthStatus::HEALTHY);

    // Test unrealistically high field
    common::Vector3 high_field(1e-3, 0.0, 0.0);  // 1 mT >> Earth field
    mag_monitor.update(high_field, 0.1);

    EXPECT_EQ(mag_monitor.getStatus(), HealthStatus::DEGRADED);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
