#include <gtest/gtest.h>
#include "fsw/core/CommandParser.hpp"
#include "fsw/telemetry/CommandProtocol.hpp"
#include "fsw/telemetry/CCSDS.hpp"
#include <vector>
#include <algorithm>

using namespace fsw;
using namespace fsw::core;
using namespace fsw::telemetry;

// Helper to encode double in Big-Endian
void encodeDoubleBE(std::vector<uint8_t>& buffer, size_t offset, double value) {
    uint8_t temp[8];
    std::memcpy(temp, &value, 8);
    std::reverse(temp, temp + 8);
    std::memcpy(buffer.data() + offset, temp, 8);
}

TEST(CommandParserTest, ParseSlewToNadir) {
    // Construct a CCSDS command packet for SLEW_TO_NADIR
    std::vector<uint8_t> packet(7, 0);
    
    // Packet ID: Type=1, APID=201
    uint16_t apid = static_cast<uint16_t>(CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    
    // Function Code: 0x01 (in Secondary Header at byte 6)
    packet[6] = static_cast<uint8_t>(GNCFunctionCode::SLEW_TO_NADIR);
    
    auto cmd = CommandParser::parse(packet);
    ASSERT_NE(cmd, nullptr);
    EXPECT_EQ(cmd->getName(), "SlewToNadir");
}

TEST(CommandParserTest, ParseSetGains) {
    std::vector<uint8_t> packet(32, 0);
    uint16_t apid = static_cast<uint16_t>(CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = static_cast<uint8_t>(GNCFunctionCode::SET_GAINS);

    double kp = 10.5, ki = 0.01, kd = 5.0;
    encodeDoubleBE(packet, 7, kp);
    encodeDoubleBE(packet, 15, ki);
    encodeDoubleBE(packet, 23, kd);
    packet[31] = 1; // is_nominal

    auto cmd = CommandParser::parse(packet);
    ASSERT_NE(cmd, nullptr);
    EXPECT_EQ(cmd->getName(), "SetPidGains");
}

TEST(CommandParserTest, ParsePointTarget) {
    std::vector<uint8_t> packet(39, 0);
    uint16_t apid = static_cast<uint16_t>(CommandAPID::GNC);
    uint16_t packet_id = (1 << 12) | (apid & 0x07FF);
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = static_cast<uint8_t>(GNCFunctionCode::POINT_TARGET);

    double qx = 0.1, qy = 0.2, qz = 0.3, qw = 0.916515;
    encodeDoubleBE(packet, 7, qx);
    encodeDoubleBE(packet, 15, qy);
    encodeDoubleBE(packet, 23, qz);
    encodeDoubleBE(packet, 31, qw);

    auto cmd = CommandParser::parse(packet);
    ASSERT_NE(cmd, nullptr);
    EXPECT_EQ(cmd->getName(), "SlewToTarget");
}

TEST(CommandParserTest, ParseInvalidType) {
    std::vector<uint8_t> packet(7, 0);
    // Type=0 (Telemetry) instead of 1 (Command)
    uint16_t packet_id = (0 << 12) | 201;
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    
    auto cmd = CommandParser::parse(packet);
    EXPECT_EQ(cmd, nullptr);
}

TEST(CommandParserTest, ParseUnknownCommand) {
    std::vector<uint8_t> packet(7, 0);
    uint16_t packet_id = (1 << 12) | 201;
    packet[0] = (packet_id >> 8) & 0xFF;
    packet[1] = packet_id & 0xFF;
    packet[6] = 0xFF; // Unknown function code
    
    auto cmd = CommandParser::parse(packet);
    EXPECT_EQ(cmd, nullptr);
}
