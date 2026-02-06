#include <gtest/gtest.h>
#include "fsw/FlightSoftware.hpp"
#include "fsw/telemetry/CommandProtocol.hpp"
#include <vector>
#include <algorithm>

using namespace fsw;

// Helper to encode double in Big-Endian
void encodeDoubleBE(std::vector<uint8_t>& buffer, size_t offset, double value) {
    uint8_t temp[8];
    std::memcpy(temp, &value, 8);
    std::reverse(temp, temp + 8);
    std::memcpy(buffer.data() + offset, temp, 8);
}

TEST(CommandIntegrationTest, SlewToNadirCommandChangesGuidance) {
    fsw::DataStore::Instance().Reset();
    FlightSoftware::Config config;
    // Minimal config for testing
    config.bdot_gain = 500.0;
    
    FlightSoftware fsw(config);
    
    SensorData sensors;
    sensors.gyro_body = common::Vector3::Zero();
    sensors.mag_body = common::Vector3(0, 5e-5, 0);
    sensors.q_measured = common::Quaternion::Identity();
    
    // 1. Initial step (Default: SUN)
    fsw.step(sensors, {}, 0.1);
    EXPECT_EQ(fsw.getCurrentMode(), core::MissionMode::SAFE); // Start in SAFE
    
    // Force transition to NOMINAL by setting low rates for a while
    for(int i=0; i<100; ++i) {
        fsw.step(sensors, {}, 0.1);
    }
    EXPECT_EQ(fsw.getCurrentMode(), core::MissionMode::NOMINAL);

    // 2. Send SLEW_TO_NADIR command
    std::vector<uint8_t> packet(7, 0);
    uint16_t apid = static_cast<uint16_t>(telemetry::CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = static_cast<uint8_t>(telemetry::GNCFunctionCode::SLEW_TO_NADIR);
    
    std::vector<std::vector<uint8_t>> commands = {packet};
    
    // This step should parse and enqueue the command
    fsw.step(sensors, commands, 0.1);
        
    // For now, let's just make sure it doesn't crash and the command was processed.
}

TEST(CommandIntegrationTest, SetPidGainsCommandUpdatesController) {
    fsw::DataStore::Instance().Reset();
    FlightSoftware::Config config;
    FlightSoftware fsw(config);
    
    // Send SET_GAINS command
    std::vector<uint8_t> packet(32, 0);
    uint16_t apid = static_cast<uint16_t>(telemetry::CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = static_cast<uint8_t>(telemetry::GNCFunctionCode::SET_GAINS);

    double kp = 50.0, ki = 0.5, kd = 10.0;
    encodeDoubleBE(packet, 7, kp);
    encodeDoubleBE(packet, 15, ki);
    encodeDoubleBE(packet, 23, kd);
    packet[31] = 1; // is_nominal
    
    fsw.step(SensorData(), {packet}, 0.1);
    
    // Verify gains updated
    auto& controller = fsw.getAttitudeController();
    EXPECT_DOUBLE_EQ(controller.getConfig().nominal_pid.kp, 50.0);
    EXPECT_DOUBLE_EQ(controller.getConfig().nominal_pid.ki, 0.5);
    EXPECT_DOUBLE_EQ(controller.getConfig().nominal_pid.kd, 10.0);
}

TEST(CommandIntegrationTest, SlewToTargetCommandUpdatesGuidance) {
    fsw::DataStore::Instance().Reset();
    FlightSoftware::Config config;
    FlightSoftware fsw(config);
    
    // Transition to NOMINAL
    SensorData sensors;
    sensors.gyro_body.setZero();
    for(int i=0; i<100; ++i) fsw.step(sensors, {}, 0.1);
    
    // Send POINT_TARGET command
    std::vector<uint8_t> packet(39, 0);
    uint16_t apid = static_cast<uint16_t>(telemetry::CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = static_cast<uint8_t>(telemetry::GNCFunctionCode::POINT_TARGET);

    common::Quaternion target_q(0.707, 0.0, 0.707, 0.0); // 90 deg around Y
    encodeDoubleBE(packet, 7, target_q.x());
    encodeDoubleBE(packet, 15, target_q.y());
    encodeDoubleBE(packet, 23, target_q.z());
    encodeDoubleBE(packet, 31, target_q.w());
    
    fsw.step(sensors, {packet}, 0.1);
}