#include "TelemetryManager.hpp"
#include "fsw/fdir/FDIRConfig.hpp"
#include <iostream>

namespace fsw {
namespace telemetry {

TelemetryManager::TelemetryManager(DataStore& ds) : ds_(ds) {}

void TelemetryManager::update(double dt) {
    attitude_timer_ += dt;
    orbit_timer_ += dt;
    health_timer_ += dt;

    std::lock_guard<std::mutex> lock(buffer_mutex_);

    // Attitude Telemetry (Quaternion and Omega)
    if (attitude_timer_ >= 1.0 / ATTITUDE_FREQ) {
        common::Quaternion q;
        common::Vector3 omega;
        if (ds_.get("attitude_estimate", q) && ds_.get("gyro_meas", omega)) {
            packet_buffer_.push_back(TelemetryService::encodeAttitude(q, omega));
        }
        attitude_timer_ = 0.0;
    }

    // Orbit Telemetry (Position and Velocity)
    if (orbit_timer_ >= 1.0 / ORBIT_FREQ) {
        common::Vector3 pos, vel;
        if (ds_.get("orbit_pos", pos) && ds_.get("orbit_vel", vel)) {
            packet_buffer_.push_back(TelemetryService::encodeOrbit(pos, vel));
        }
        orbit_timer_ = 0.0;
    }

    // Health Telemetry
    if (health_timer_ >= 1.0 / HEALTH_FREQ) {
        // Example: Monitor Gyro Health
        fdir::HealthStatus status;
        if (ds_.get("gyro_health", status)) {
            packet_buffer_.push_back(TelemetryService::encodeHealth("GYRO", status));
        }
        health_timer_ = 0.0;
    }
}

std::vector<std::vector<uint8_t>> TelemetryManager::flushPackets() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    std::vector<std::vector<uint8_t>> temp;
    temp.swap(packet_buffer_);
    return temp;
}

} // namespace telemetry
} // namespace fsw
