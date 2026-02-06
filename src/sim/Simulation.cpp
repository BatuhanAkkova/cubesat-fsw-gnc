#include "sim/Simulation.hpp"
#include <iostream>

namespace sim {

Simulation::Simulation(const Config& config) {
    body_ = std::make_unique<dynamics::RigidBody>(config.inertia, config.init_q, config.init_w);
    orbit_ = std::make_unique<dynamics::Orbit>(config.init_pos, config.init_vel, config.orbit_cfg);
    gyro_ = std::make_unique<models::SimGyro>(*body_, config.gyro_cfg);
    mag_ = std::make_unique<models::SimMagnetometer>(*body_, *orbit_, config.mag_cfg);
    st_ = std::make_unique<models::SimStarTracker>(*body_, 0.0);
}

void Simulation::step(double dt, const common::Vector3& torque_cmd) {
    sim_time_ += dt;
    
    // Update orbit
    orbit_->step(dt);
    
    // Update magnetic field time
    mag_->setTime(sim_time_);
    
    // Update dynamics (external torque only for now, can add internal components later)
    body_->step(dt, torque_cmd);
}

fsw::SensorData Simulation::getSensors() const {
    fsw::SensorData sensors;
    sensors.mag_body = mag_->read();
    sensors.gyro_body = gyro_->read();
    sensors.q_measured = st_->getOrientation();
    
    // Compute Sun vector in body frame (simplified: Sun is at [1, 0, 0] in Inertial)
    common::Vector3 sun_inertial(1, 0, 0); 
    sensors.sun_body = body_->getAttitude().conjugate() * sun_inertial;
    
    return sensors;
}

} // namespace sim
