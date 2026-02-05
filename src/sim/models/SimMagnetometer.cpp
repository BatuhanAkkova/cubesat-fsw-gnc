#include "SimMagnetometer.hpp"
#include <cmath>

namespace sim {
namespace models {

SimMagnetometer::SimMagnetometer(const dynamics::RigidBody& body, const dynamics::Orbit& orbit)
    : body_(body), orbit_(orbit) {}

common::Vector3 SimMagnetometer::read() {
    common::Vector3 pos_eci = orbit_.getPosition();
    common::Vector3 b_eci = computeDipoleField(pos_eci);

    // Rotate ECI vector to Body frame
    // q_b_to_i * v_b = v_i => v_b = q_b_to_i.inverse() * v_i
    common::Quaternion q_b_to_i = body_.getAttitude();
    common::Vector3 b_body = q_b_to_i.inverse() * b_eci;

    return b_body;
}

common::Vector3 SimMagnetometer::computeDipoleField(const common::Vector3& pos_eci) const {
    // Tilted dipole model constants (Approximate)
    const double B0 = 3.12e-5; // Tesla
    const double RE = 6371000.0; // Earth Radius [m]
    
    // For simplicity, assume Dipole is aligned with Z-axis (Rotational Axis) for now (Untilted)
    // In a real simulation, we'd use the epoch time to rotate the Earth fixed dipole into ECI.
    // Here we assume ECI ~= ECEF (no earth rotation) or Dipole is axially symmetric
    // Actually, let's just implement a simple axial dipole.
    
    double r = pos_eci.norm();
    common::Vector3 r_hat = pos_eci.normalized();
    common::Vector3 z_hat(0, 0, 1);

    double factor = B0 * std::pow(RE / r, 3);
    
    // Dipole formula: B = B0 * (RE/r)^3 * [3(m . r_hat)r_hat - m]
    // Let's assume magnetic dipole m is aligned with -Z axis (approx).
    common::Vector3 m = -z_hat; 

    common::Vector3 B = factor * (3.0 * (m.dot(r_hat)) * r_hat - m);
    return B;
}

} // namespace models
} // namespace sim
