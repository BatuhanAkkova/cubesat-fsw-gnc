#pragma once

#include <memory>
#include <vector>

#include "common/types.hpp"
#include "fsw/FlightSoftware.hpp"
#include "sim/dynamics/Orbit.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimGyro.hpp"
#include "sim/models/SimMagnetometer.hpp"
#include "sim/models/SimRW.hpp"
#include "sim/models/SimStarTracker.hpp"

namespace sim {

/**
 * @brief Configuration for simulation
 */
struct SimulationConfig {
    common::Matrix3 inertia = common::Matrix3::Identity() * 0.01;
    common::Quaternion init_q = common::Quaternion::Identity();
    common::Vector3 init_w = common::Vector3::Zero();

    common::Vector3 init_pos = common::Vector3(7000000, 0, 0);  // ~600km LEO
    common::Vector3 init_vel = common::Vector3(0, 7500, 0);

    dynamics::Orbit::Config orbit_cfg;
    models::SimGyro::Config gyro_cfg;
    models::SimMagnetometer::Config mag_cfg;
};

/**
 * @brief Simulation wrapper to simplify running environmental simulations.
 * Manages dynamics, orbit, and sensors.
 */
class Simulation {
   public:
    using Config = SimulationConfig;

    Simulation(const Config& config);

    /**
     * @brief Step the whole simulation by dt.
     * @param torque_cmd External torque from FSW (Body frame) [Nm]
     */
    void step(double dt, const common::Vector3& torque_cmd);

    /**
     * @brief Get sensor readings for FSW.
     */
    common::SensorData getSensors() const;

    // Truth accessors
    common::Quaternion getAttitude() const {
        return body_->getAttitude();
    }
    common::Vector3 getAngularRate() const {
        return body_->getAngularVelocity();
    }
    common::Vector3 getPosition() const {
        return orbit_->getPosition();
    }

    // For fault injection
    models::SimGyro& getGyro() {
        return *gyro_;
    }
    models::SimMagnetometer& getMag() {
        return *mag_;
    }
    models::SimStarTracker& getStarTracker() {
        return *st_;
    }

   private:
    std::unique_ptr<dynamics::RigidBody> body_;
    std::unique_ptr<dynamics::Orbit> orbit_;
    std::unique_ptr<models::SimGyro> gyro_;
    std::unique_ptr<models::SimMagnetometer> mag_;
    std::unique_ptr<models::SimStarTracker> st_;

    double sim_time_ = 0.0;
};

}  // namespace sim
