#include "SimMagnetometer.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sim {
namespace models {

// Earth rotation rate [rad/s]
static constexpr double OMEGA_EARTH = 7.2921159e-5;

// Magnetic dipole tilt angle from rotation axis [rad]
static constexpr double DIPOLE_TILT = 11.0 * M_PI / 180.0;

SimMagnetometer::SimMagnetometer(const dynamics::RigidBody& body, const dynamics::Orbit& orbit,
                                 const Config& config)
    : body_(body), orbit_(orbit), config_(config), sim_time_(0.0) {}


void SimMagnetometer::setTime(double time_sec) {
    sim_time_ = time_sec;
}

common::Vector3 SimMagnetometer::read() {
    common::Vector3 pos_eci = orbit_.getPosition();
    common::Vector3 b_eci = computeDipoleField(pos_eci, sim_time_);

    // Rotate ECI vector to Body frame
    // q_b_to_i * v_b = v_i => v_b = q_b_to_i.inverse() * v_i
    common::Quaternion q_b_to_i = body_.getAttitude();
    common::Vector3 b_body = q_b_to_i.inverse() * b_eci;

    return b_body;
}


common::Vector3 SimMagnetometer::computeDipoleField(const common::Vector3& pos_eci, double time_sec) const {
    // Tilted dipole model constants
    const double B0 = 3.12e-5;      // Tesla (magnetic field strength at equator at Earth's surface)
    const double RE = 6371000.0;    // Earth Radius [m]
    
    double r = pos_eci.norm();
    common::Vector3 r_hat = pos_eci.normalized();
    
    // Magnetic dipole moment direction in ECI
    common::Vector3 m;
    
    if (config_.enable_earth_rotation) {
        // Earth rotates, so dipole axis rotates in ECI frame
        // Rotation angle = OMEGA_EARTH * time
        double theta = OMEGA_EARTH * time_sec;
        
        if (config_.use_tilted_dipole) {
            // Tilted dipole: 11-degree offset from rotation axis
            // Dipole starts aligned with -Z, tilted in XZ plane
            // Then rotates about Z-axis with Earth rotation
            double cos_theta = std::cos(theta);
            double sin_theta = std::sin(theta);
            double cos_tilt = std::cos(DIPOLE_TILT);
            double sin_tilt = std::sin(DIPOLE_TILT);
            
            // Magnetic dipole direction (unit vector)
            // Rotated about Z-axis by theta, tilted by DIPOLE_TILT in XZ plane
            m.x() = -sin_tilt * cos_theta;
            m.y() = -sin_tilt * sin_theta;
            m.z() = -cos_tilt;
        } else {
            // Simple case: dipole aligned with Z-axis, rotates with Earth
            // Since dipole is axially symmetric about Z, rotation doesn't change it
            m = common::Vector3(0, 0, -1);
        }
    } else {
        // Static dipole aligned with -Z axis
        m = common::Vector3(0, 0, -1);
    }
    
    double factor = B0 * std::pow(RE / r, 3);
    
    // Dipole formula: B = B0 * (RE/r)^3 * [3(m . r_hat)r_hat - m]
    common::Vector3 B = factor * (3.0 * (m.dot(r_hat)) * r_hat - m);
    
    return B;
}

} // namespace models
} // namespace sim
