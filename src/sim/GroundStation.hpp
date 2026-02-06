#pragma once

#include "fsw/telemetry/CCSDS.hpp"
#include "fsw/telemetry/CommandProtocol.hpp"
#include "fsw/fdir/FDIRConfig.hpp"
#include "common/types.hpp"
#include <vector>
#include <string>
#include <map>

namespace sim {

/**
 * @brief Ground Station Simulation class.
 * Responsible for encoding commands and decoding telemetry.
 */
class GroundStation {
public:
    struct AttitudeTelemetry {
        common::Quaternion q;
        common::Vector3 omega;
    };

    struct OrbitTelemetry {
        common::Vector3 position;
        common::Vector3 velocity;
    };

    struct HealthTelemetry {
        fsw::fdir::HealthStatus status;
        std::string sensor_name;
    };

    GroundStation();

    // --- Command Encoding ---
    
    /**
     * @brief Create a Slew to Nadir command packet.
     */
    std::vector<uint8_t> createSlewToNadirCommand();

    /**
     * @brief Create a Slew to Target command packet.
     * @param target_q Target orientation.
     */
    std::vector<uint8_t> createSlewToTargetCommand(const common::Quaternion& target_q);

    /**
     * @brief Create a Set PID Gains command packet.
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param is_nominal Whether these are nominal or backup gains.
     */
    std::vector<uint8_t> createSetPidGainsCommand(double kp, double ki, double kd, bool is_nominal);

    // --- Telemetry Decoding ---

    /**
     * @brief Process a list of received telemetry packets.
     * @param packets Raw CCSDS telemetry packets.
     */
    void processTelemetry(const std::vector<std::vector<uint8_t>>& packets);

    // --- Accessors ---
    const AttitudeTelemetry& getLatestAttitude() const { return last_attitude_; }
    const OrbitTelemetry& getLatestOrbit() const { return last_orbit_; }
    const std::map<std::string, HealthTelemetry>& getHealthMap() const { return health_map_; }

private:
    /**
     * @brief Helper to initialize a CCSDS command header.
     */
    fsw::telemetry::CCSDSHeader createCommandHeader(fsw::telemetry::CommandAPID apid, uint16_t payload_size);

    /**
     * @brief Helper to append a double in Big Endian to a buffer.
     */
    void appendDouble(std::vector<uint8_t>& buffer, double value);

    /**
     * @brief Helper to read a double in Big Endian from a buffer.
     */
    double readDouble(const uint8_t* data, size_t& offset);

    AttitudeTelemetry last_attitude_;
    OrbitTelemetry last_orbit_;
    std::map<std::string, HealthTelemetry> health_map_;

    uint16_t command_sequence_count_ = 0;
};

} // namespace sim
