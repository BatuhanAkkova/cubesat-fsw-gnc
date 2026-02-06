#include <gtest/gtest.h>
#include "fsw/telemetry/TelemetryService.hpp"
#include "fsw/telemetry/CCSDS.hpp"
#include "common/types.hpp"
#include <iostream>
#include <iomanip>

using namespace fsw::telemetry;

TEST(TelemetryTest, CCSDSHeaderEncoding) {
    CCSDSHeader header;
    header.packet_id = 0;
    header.sequence_ctrl = 0;
    
    header.setVersion(0);
    header.setType(0);
    header.setSecondaryHeaderFlag(false);
    header.setAPID(100);
    header.setSequenceFlags(0x03);
    header.setSequenceCount(123);
    header.setLength(64);

    header.toNetworkOrder();

    // Verify manual bit manipulation (Big Endian results)
    uint8_t* raw = reinterpret_cast<uint8_t*>(&header);
    
    // Packet ID: (0 << 13) | (0 << 12) | (0 << 11) | 100 = 100 (0x0064)
    // Big Endian 0x0064 -> [0x00, 0x64]
    EXPECT_EQ(raw[0], 0x00);
    EXPECT_EQ(raw[1], 0x64);

    // Sequence Ctrl: (3 << 14) | 123 = 0xC07B
    // Big Endian 0xC07B -> [0xC0, 0x7B]
    EXPECT_EQ(raw[2], 0xC0);
    EXPECT_EQ(raw[3], 0x7B);

    // Length: 64 - 1 = 63 (0x003F)
    // Big Endian 0x3F -> [0x00, 0x3F]
    EXPECT_EQ(raw[4], 0x00);
    EXPECT_EQ(raw[5], 0x3F);
}

TEST(TelemetryTest, EncodeAttitude) {
    common::Quaternion q(1.0, 0.0, 0.0, 0.0);
    common::Vector3 omega(0.1, 0.2, 0.3);

    auto packet = TelemetryService::encodeAttitude(q, omega);

    EXPECT_EQ(packet.size(), 6 + 56);
    
    // Check APID in header (Attitude = 100 = 0x64)
    EXPECT_EQ(packet[0], 0x00);
    EXPECT_EQ(packet[1], 0x64);

    // Check Payload (Manual check of first double q.w = 1.0)
    // 1.0 in IEEE 754 Big Endian is 0x3F F0 00 00 00 00 00 00
    EXPECT_EQ(packet[6], 0x3F);
    EXPECT_EQ(packet[7], 0xF0);
}

TEST(TelemetryTest, EncodeHealth) {
    std::string name = "IMU";
    auto packet = TelemetryService::encodeHealth(name, fsw::fdir::HealthStatus::DEGRADED);

    EXPECT_EQ(packet.size(), 6 + 2 + 3);
    EXPECT_EQ(packet[6], static_cast<uint8_t>(fsw::fdir::HealthStatus::DEGRADED));
    EXPECT_EQ(packet[7], 3);
    EXPECT_EQ(packet[8], 'I');
    EXPECT_EQ(packet[9], 'M');
    EXPECT_EQ(packet[10], 'U');
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
