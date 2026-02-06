#include "sim/GroundStation.hpp"
#include <iostream>
#include <algorithm>
#include <cstring>

namespace sim {

GroundStation::GroundStation() {
    last_attitude_.q = common::Quaternion::Identity();
    last_attitude_.omega = common::Vector3::Zero();
    last_orbit_.position = common::Vector3::Zero();
    last_orbit_.velocity = common::Vector3::Zero();
}

fsw::telemetry::CCSDSHeader GroundStation::createCommandHeader(fsw::telemetry::CommandAPID apid, uint16_t payload_size) {
    fsw::telemetry::CCSDSHeader header;
    header.packet_id = 0;
    header.sequence_ctrl = 0;
    
    header.setVersion(0);
    header.setType(1); // 1 for Command
    header.setSecondaryHeaderFlag(true);
    header.setAPID(static_cast<uint16_t>(apid));
    header.setSequenceFlags(0x03); // Standalone packet
    header.setSequenceCount(command_sequence_count_++);
    header.setLength(6 + sizeof(fsw::telemetry::CommandSecondaryHeader) + payload_size);

    header.toNetworkOrder();
    return header;
}

void GroundStation::appendDouble(std::vector<uint8_t>& buffer, double value) {
    uint8_t temp[8];
    std::memcpy(temp, &value, 8);
    
    // CCSDS is Big-Endian. Reverse if host is Little-Endian.
    // For simplicity, we reverse as our FSW TelemetryService does.
    for(int i = 0; i < 4; ++i) {
        uint8_t t = temp[i];
        temp[i] = temp[7-i];
        temp[7-i] = t;
    }
    
    buffer.insert(buffer.end(), temp, temp + 8);
}

double GroundStation::readDouble(const uint8_t* data, size_t& offset) {
    uint8_t temp[8];
    std::memcpy(temp, data + offset, 8);
    
    // Reverse from Big Endian
    for(int i = 0; i < 4; ++i) {
        uint8_t t = temp[i];
        temp[i] = temp[7-i];
        temp[7-i] = t;
    }
    
    double val;
    std::memcpy(&val, temp, 8);
    offset += 8;
    return val;
}

std::vector<uint8_t> GroundStation::createSlewToNadirCommand() {
    fsw::telemetry::CCSDSHeader header = createCommandHeader(fsw::telemetry::CommandAPID::GNC, 0);
    fsw::telemetry::CommandSecondaryHeader sec_header;
    sec_header.function_code = static_cast<uint8_t>(fsw::telemetry::GNCFunctionCode::SLEW_TO_NADIR);

    std::vector<uint8_t> packet(6 + sizeof(fsw::telemetry::CommandSecondaryHeader));
    std::memcpy(packet.data(), &header, 6);
    std::memcpy(packet.data() + 6, &sec_header, sizeof(fsw::telemetry::CommandSecondaryHeader));
    
    return packet;
}

std::vector<uint8_t> GroundStation::createSlewToTargetCommand(const common::Quaternion& target_q) {
    fsw::telemetry::CCSDSHeader header = createCommandHeader(fsw::telemetry::CommandAPID::GNC, 32);
    fsw::telemetry::CommandSecondaryHeader sec_header;
    sec_header.function_code = static_cast<uint8_t>(fsw::telemetry::GNCFunctionCode::POINT_TARGET);

    std::vector<uint8_t> packet(6 + sizeof(fsw::telemetry::CommandSecondaryHeader));
    std::memcpy(packet.data(), &header, 6);
    std::memcpy(packet.data() + 6, &sec_header, sizeof(fsw::telemetry::CommandSecondaryHeader));

    appendDouble(packet, target_q.x());
    appendDouble(packet, target_q.y());
    appendDouble(packet, target_q.z());
    appendDouble(packet, target_q.w());
    
    return packet;
}

std::vector<uint8_t> GroundStation::createSetPidGainsCommand(double kp, double ki, double kd, bool is_nominal) {
    fsw::telemetry::CCSDSHeader header = createCommandHeader(fsw::telemetry::CommandAPID::GNC, 25);
    fsw::telemetry::CommandSecondaryHeader sec_header;
    sec_header.function_code = static_cast<uint8_t>(fsw::telemetry::GNCFunctionCode::SET_GAINS);

    std::vector<uint8_t> packet(6 + sizeof(fsw::telemetry::CommandSecondaryHeader));
    std::memcpy(packet.data(), &header, 6);
    std::memcpy(packet.data() + 6, &sec_header, sizeof(fsw::telemetry::CommandSecondaryHeader));

    appendDouble(packet, kp);
    appendDouble(packet, ki);
    appendDouble(packet, kd);
    packet.push_back(is_nominal ? 1 : 0);
    
    return packet;
}

void GroundStation::processTelemetry(const std::vector<std::vector<uint8_t>>& packets) {
    for (const auto& raw_packet : packets) {
        if (raw_packet.size() < 6) continue;

        // Parse CCSDS Header (Big Endian)
        uint16_t packet_id = (raw_packet[0] << 8) | raw_packet[1];
        uint16_t apid_val = packet_id & 0x07FF;
        uint8_t type = (packet_id >> 12) & 0x01;

        if (type != 0) continue; // Not telemetry

        size_t payload_offset = 6;
        if (apid_val == static_cast<uint16_t>(fsw::telemetry::APID::ATTITUDE)) {
            if (raw_packet.size() < 6 + 56) continue;
            last_attitude_.q.w() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.q.x() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.q.y() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.q.z() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.omega.x() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.omega.y() = readDouble(raw_packet.data(), payload_offset);
            last_attitude_.omega.z() = readDouble(raw_packet.data(), payload_offset);
        }
        else if (apid_val == static_cast<uint16_t>(fsw::telemetry::APID::ORBIT)) {
            if (raw_packet.size() < 6 + 48) continue;
            last_orbit_.position.x() = readDouble(raw_packet.data(), payload_offset);
            last_orbit_.position.y() = readDouble(raw_packet.data(), payload_offset);
            last_orbit_.position.z() = readDouble(raw_packet.data(), payload_offset);
            last_orbit_.velocity.x() = readDouble(raw_packet.data(), payload_offset);
            last_orbit_.velocity.y() = readDouble(raw_packet.data(), payload_offset);
            last_orbit_.velocity.z() = readDouble(raw_packet.data(), payload_offset);
        }
        else if (apid_val == static_cast<uint16_t>(fsw::telemetry::APID::HEALTH)) {
            if (raw_packet.size() < 6 + 2) continue;
            fsw::fdir::HealthStatus status = static_cast<fsw::fdir::HealthStatus>(raw_packet[6]);
            uint8_t name_len = raw_packet[7];
            if (raw_packet.size() < 8 + name_len) continue;
            
            std::string name(reinterpret_cast<const char*>(raw_packet.data() + 8), name_len);
            
            HealthTelemetry ht;
            ht.status = status;
            ht.sensor_name = name;
            health_map_[name] = ht;
        }
    }
}

} // namespace sim
