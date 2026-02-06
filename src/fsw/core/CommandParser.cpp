#include "fsw/core/CommandParser.hpp"
#include "fsw/telemetry/CommandProtocol.hpp"
#include "fsw/telemetry/CommandParameter.hpp"
#include "fsw/core/commands/SlewToNadirCommand.hpp"
#include "fsw/core/commands/SlewToTargetCommand.hpp"
#include "fsw/core/commands/SetPidGainsCommand.hpp"
#include "common/logger.hpp"

namespace fsw {
namespace core {

std::unique_ptr<Command> CommandParser::parse(const std::vector<uint8_t>& raw_packet) {
    if (raw_packet.size() < sizeof(telemetry::CCSDSHeader)) {
        common::LogError("Packet too small for CCSDS header");
        return nullptr;
    }
    
    // Extract APID and Type
    uint16_t packet_id = (raw_packet[0] << 8) | raw_packet[1];
    uint16_t apid = packet_id & 0x07FF;
    uint8_t type = (packet_id >> 12) & 0x01;

    if (type != 1) { // 1 = Command/Telecommand
        common::LogError("Packet is not a command packet (Type={})", type);
        return nullptr;
    }

    // Packet payload starts after the 6-byte header
    if (raw_packet.size() < sizeof(telemetry::CCSDSHeader) + sizeof(telemetry::CommandSecondaryHeader)) {
        common::LogError("Packet too small for command secondary header");
        return nullptr;
    }

    const telemetry::CommandSecondaryHeader* sec_header = 
        reinterpret_cast<const telemetry::CommandSecondaryHeader*>(raw_packet.data() + sizeof(telemetry::CCSDSHeader));
    
    uint8_t function_code = sec_header->function_code;
    
    telemetry::CommandParameterReader reader(
        raw_packet.data() + sizeof(telemetry::CCSDSHeader) + sizeof(telemetry::CommandSecondaryHeader),
        raw_packet.size() - sizeof(telemetry::CCSDSHeader) - sizeof(telemetry::CommandSecondaryHeader)
    );

    if (apid == static_cast<uint16_t>(telemetry::CommandAPID::GNC)) {
        if (function_code == static_cast<uint8_t>(telemetry::GNCFunctionCode::SLEW_TO_NADIR)) {
            return std::make_unique<commands::SlewToNadirCommand>();
        }
        else if (function_code == static_cast<uint8_t>(telemetry::GNCFunctionCode::SET_GAINS)) {
            if (reader.getRemainingSize() < 25) {
                common::LogError("SET_GAINS payload too small: {} bytes", reader.getRemainingSize());
                return nullptr;
            }
            double kp = reader.read<double>();
            double ki = reader.read<double>();
            double kd = reader.read<double>();
            bool is_nominal = reader.readBool();
            return std::make_unique<commands::SetPidGainsCommand>(kp, ki, kd, is_nominal);
        }
        else if (function_code == static_cast<uint8_t>(telemetry::GNCFunctionCode::POINT_TARGET)) {
            if (reader.getRemainingSize() < 32) {
                common::LogError("POINT_TARGET payload too small: {} bytes", reader.getRemainingSize());
                return nullptr;
            }
            double qx = reader.read<double>();
            double qy = reader.read<double>();
            double qz = reader.read<double>();
            double qw = reader.read<double>();
            return std::make_unique<commands::SlewToTargetCommand>(common::Quaternion(qw, qx, qy, qz));
        }
    }

    common::LogWarning("Unknown command: APID={}, FunctionCode={}", apid, function_code);
    return nullptr;
}

} // namespace core
} // namespace fsw
