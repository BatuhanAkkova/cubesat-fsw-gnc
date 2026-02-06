#pragma once

#include "CCSDS.hpp"
#include "common/types.hpp"
#include "fsw/fdir/FDIRConfig.hpp"
#include <vector>
#include <string>

namespace fsw {
namespace telemetry {

/**
 * @brief Service to encode FSW data into CCSDS packets.
 */
class TelemetryService {
public:
    /**
     * @brief Encode attitude data (quaternion and angular velocity)
     */
    static std::vector<uint8_t> encodeAttitude(const common::Quaternion& q, const common::Vector3& omega);

    /**
     * @brief Encode orbit data (position and velocity in ECI)
     */
    static std::vector<uint8_t> encodeOrbit(const common::Vector3& pos, const common::Vector3& vel);

    /**
     * @brief Encode health status data
     */
    static std::vector<uint8_t> encodeHealth(const std::string& sensor_name, fdir::HealthStatus status);

private:
    static std::vector<uint8_t> preparePacket(APID apid, uint16_t payload_size);
    static void appendDouble(std::vector<uint8_t>& buffer, double value);
    static uint16_t sequence_counts_[2048]; // One for each APID
};

} // namespace telemetry
} // namespace fsw
