#pragma once

#include <mutex>
#include <vector>

#include "TelemetryService.hpp"
#include "fsw/core/DataStore.hpp"

namespace fsw {
namespace telemetry {

/**
 * @brief Manager to collect data from DataStore and generate telemetry packets.
 */
class TelemetryManager {
   public:
    TelemetryManager(DataStore& ds);

    /**
     * @brief Periodic update to generate telemetry
     */
    void update(double dt);

    /**
     * @brief Get all generated packets and clear the buffer
     */
    std::vector<std::vector<uint8_t>> flushPackets();

   private:
    DataStore& ds_;
    std::vector<std::vector<uint8_t>> packet_buffer_;
    std::mutex buffer_mutex_;

    double attitude_timer_ = 0.0;
    double orbit_timer_ = 0.0;
    double health_timer_ = 0.0;

    const double ATTITUDE_FREQ = 10.0;  // Hz
    const double ORBIT_FREQ = 1.0;      // Hz
    const double HEALTH_FREQ = 0.5;     // Hz
};

}  // namespace telemetry
}  // namespace fsw
