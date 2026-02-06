#include <gtest/gtest.h>
#include "sim/GroundStation.hpp"
#include "fsw/telemetry/TelemetryService.hpp"
#include "common/types.hpp"

using namespace sim;
using namespace fsw::telemetry;

TEST(GroundStationTest, CreateSlewToNadirCommand) {
    GroundStation gs;
    auto packet = gs.createSlewToNadirCommand();

    // CCSDS Header (6 bytes) + Secondary Header (1 byte function code)
    ASSERT_EQ(packet.size(), 7);

    // APID for GNC is 201 (0x00C9 in Big Endian, but bit fields are involved)
    // APID is lower 11 bits. Type is 1 (Command).
    // Packet ID = (Version:0 << 13) | (Type:1 << 12) | (SecHdr:1 << 11) | 201
    // Packet ID = 0x1000 | 0x0800 | 201 = 0x1800 | 0xC9 = 0x18C9
    // Big Endian: [0x18, 0xC9]
    EXPECT_EQ(packet[0], 0x18);
    EXPECT_EQ(packet[1], 0xC9);

    // Function code for SLEW_TO_NADIR is 0x01
    EXPECT_EQ(packet[6], 0x01);
}

TEST(GroundStationTest, CreateSetPidGainsCommand) {
    GroundStation gs;
    double kp = 1.23, ki = 4.56, kd = 7.89;
    bool is_nominal = true;

    auto packet = gs.createSetPidGainsCommand(kp, ki, kd, is_nominal);

    // 6 (CCSDS) + 1 (Function Code) + 3*8 (doubles) + 1 (bool) = 32
    ASSERT_EQ(packet.size(), 32);

    // Function code for SET_GAINS is 0x03
    EXPECT_EQ(packet[6], 0x03);

    // Verify boolean at the end
    EXPECT_EQ(packet[31], 1);
}

TEST(GroundStationTest, ProcessTelemetry) {
    GroundStation gs;
    
    // Create mock telemetry using TelemetryService
    common::Quaternion q(0.707, 0.0, 0.707, 0.0);
    common::Vector3 omega(0.01, -0.02, 0.03);
    auto att_packet = TelemetryService::encodeAttitude(q, omega);

    common::Vector3 pos(7000e3, 0, 0);
    common::Vector3 vel(0, 7500, 0);
    auto orbit_packet = TelemetryService::encodeOrbit(pos, vel);

    auto health_packet = TelemetryService::encodeHealth("StarTracker_1", fsw::fdir::HealthStatus::HEALTHY);

    std::vector<std::vector<uint8_t>> packets = {att_packet, orbit_packet, health_packet};
    
    gs.processTelemetry(packets);

    // Verify decoded attitude
    EXPECT_NEAR(gs.getLatestAttitude().q.w(), q.w(), 1e-6);
    EXPECT_NEAR(gs.getLatestAttitude().q.x(), q.x(), 1e-6);
    EXPECT_NEAR(gs.getLatestAttitude().omega.y(), omega.y(), 1e-6);

    // Verify decoded orbit
    EXPECT_NEAR(gs.getLatestOrbit().position.x(), pos.x(), 1e-3);
    EXPECT_NEAR(gs.getLatestOrbit().velocity.y(), vel.y(), 1e-3);

    // Verify decoded health
    auto health_map = gs.getHealthMap();
    ASSERT_TRUE(health_map.count("StarTracker_1"));
    EXPECT_EQ(health_map["StarTracker_1"].status, fsw::fdir::HealthStatus::HEALTHY);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
