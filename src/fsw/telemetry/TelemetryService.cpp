#include "TelemetryService.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace fsw {
namespace telemetry {

uint16_t TelemetryService::sequence_counts_[2048] = {0};

std::vector<uint8_t> TelemetryService::preparePacket(APID apid, uint16_t payload_size) {
    CCSDSHeader header;
    header.packet_id = 0;
    header.sequence_ctrl = 0;

    header.setVersion(0);
    header.setType(0);  // 0 for TLM
    header.setSecondaryHeaderFlag(false);
    header.setAPID(static_cast<uint16_t>(apid));
    header.setSequenceFlags(0x03);  // Standalone packet
    header.setSequenceCount(sequence_counts_[static_cast<uint16_t>(apid) & 0x07FF]++);
    header.setLength(6 + payload_size);  // 6 bytes of header + payload

    header.toNetworkOrder();

    std::vector<uint8_t> packet(6 + payload_size);
    std::memcpy(packet.data(), &header, 6);
    return packet;
}

void TelemetryService::appendDouble(std::vector<uint8_t>& buffer, double value) {
    uint8_t temp[8];
    std::memcpy(temp, &value, 8);

    // Reverse for Big Endian (Standard CCSDS)
    for (int i = 0; i < 4; ++i) {
        uint8_t t = temp[i];
        temp[i] = temp[7 - i];
        temp[7 - i] = t;
    }

    buffer.insert(buffer.end(), temp, temp + 8);
}

std::vector<uint8_t> TelemetryService::encodeAttitude(const common::Quaternion& q, const common::Vector3& omega) {
    std::vector<uint8_t> packet = preparePacket(APID::ATTITUDE, 56);
    std::vector<uint8_t> payload;
    payload.reserve(56);

    appendDouble(payload, q.w());
    appendDouble(payload, q.x());
    appendDouble(payload, q.y());
    appendDouble(payload, q.z());
    appendDouble(payload, omega.x());
    appendDouble(payload, omega.y());
    appendDouble(payload, omega.z());

    std::memcpy(packet.data() + 6, payload.data(), 56);
    return packet;
}

std::vector<uint8_t> TelemetryService::encodeOrbit(const common::Vector3& pos, const common::Vector3& vel) {
    std::vector<uint8_t> packet = preparePacket(APID::ORBIT, 48);
    std::vector<uint8_t> payload;
    payload.reserve(48);

    appendDouble(payload, pos.x());
    appendDouble(payload, pos.y());
    appendDouble(payload, pos.z());
    appendDouble(payload, vel.x());
    appendDouble(payload, vel.y());
    appendDouble(payload, vel.z());

    std::memcpy(packet.data() + 6, payload.data(), 48);
    return packet;
}

std::vector<uint8_t> TelemetryService::encodeHealth(const std::string& sensor_name, fdir::HealthStatus status) {
    uint8_t name_len = static_cast<uint8_t>(std::min(sensor_name.length(), static_cast<size_t>(255)));
    std::vector<uint8_t> packet = preparePacket(APID::HEALTH, 2 + name_len);

    packet[6] = static_cast<uint8_t>(status);
    packet[7] = name_len;
    std::memcpy(packet.data() + 8, sensor_name.c_str(), name_len);

    return packet;
}

}  // namespace telemetry
}  // namespace fsw
