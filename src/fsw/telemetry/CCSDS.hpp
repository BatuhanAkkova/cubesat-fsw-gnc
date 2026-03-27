#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace fsw {
namespace telemetry {

/**
 * @brief CCSDS Primary Header (6 bytes)
 * Based on CCSDS 133.0-B-1 (Space Packet Protocol)
 *
 * Note: CCSDS headers are Big-endian.
 */
struct CCSDSHeader {
    uint16_t packet_id;      // Version (3b), Type (1b), Sec. Hdr Flag (1b), APID (11b)
    uint16_t sequence_ctrl;  // Sequence Flags (2b), Sequence Count (14b)
    uint16_t packet_length;  // Length (16b) = Total bytes - 7

    // Helpers to set/get fields (handling bitfields manually for endianness safety)

    void setVersion(uint8_t version) {
        packet_id = (packet_id & 0x1FFF) | ((version & 0x07) << 13);
    }

    void setType(uint8_t type) {
        packet_id = (packet_id & 0xEFFF) | ((type & 0x01) << 12);
    }

    void setSecondaryHeaderFlag(bool flag) {
        packet_id = (packet_id & 0xF7FF) | (flag ? 0x0800 : 0x0000);
    }

    void setAPID(uint16_t apid) {
        packet_id = (packet_id & 0xF800) | (apid & 0x07FF);
    }

    void setSequenceFlags(uint8_t flags) {
        sequence_ctrl = (sequence_ctrl & 0x3FFF) | ((flags & 0x03) << 14);
    }

    void setSequenceCount(uint16_t count) {
        sequence_ctrl = (sequence_ctrl & 0xC000) | (count & 0x3FFF);
    }

    void setLength(uint16_t length) {
        packet_length = length - 1;  // CCSDS length is (total payload bytes + secondary header - 1)
    }

    // Convert to network byte order (Big Endian)
    void toNetworkOrder() {
        packet_id = ((packet_id & 0xFF00) >> 8) | ((packet_id & 0x00FF) << 8);
        sequence_ctrl = ((sequence_ctrl & 0xFF00) >> 8) | ((sequence_ctrl & 0x00FF) << 8);
        packet_length = ((packet_length & 0xFF00) >> 8) | ((packet_length & 0x00FF) << 8);
    }
};

// APID Definitions
enum class APID : uint16_t { ATTITUDE = 100, ORBIT = 101, HEALTH = 102 };

/**
 * @brief CCSDS Command Secondary Header
 * Usually contains the function code for the command.
 */
struct CommandSecondaryHeader {
    uint8_t function_code;
    // Add more fields if needed (e.g. checksum, time tag)
};

}  // namespace telemetry
}  // namespace fsw
