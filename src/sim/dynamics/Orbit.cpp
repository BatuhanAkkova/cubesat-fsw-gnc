#include "Orbit.hpp"
#include "sim/engine/Integrator.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sim {
namespace dynamics {

// Standard gravitational parameter for Earth [m^3/s^2]
static constexpr double MU_EARTH = 3.986004418e14;

// J2 oblateness coefficient (dimensionless)
static constexpr double J2_EARTH = 1.08263e-3;

// Earth equatorial radius [m]
static constexpr double R_EARTH = 6378137.0;

Orbit::Orbit(const common::Vector3& init_pos, const common::Vector3& init_vel, 
             const Config& config) 
    : config_(config) {
    state_.resize(6);
    state_.segment<3>(0) = init_pos;
    state_.segment<3>(3) = init_vel;
}

common::VectorX Orbit::dynamics(double, const common::VectorX& y) {
    common::Vector3 pos = y.segment<3>(0);
    common::Vector3 vel = y.segment<3>(3);

    double r_norm = pos.norm();
    double mu_r3 = MU_EARTH / (std::pow(r_norm, 3));

    // Two-body Keplerian acceleration
    common::Vector3 acc = -mu_r3 * pos;

    // Add J2 perturbation if enabled
    if (config_.enable_j2) {
        acc += computeJ2Acceleration(pos);
    }

    common::VectorX dydt(6);
    dydt.segment<3>(0) = vel;
    dydt.segment<3>(3) = acc;

    return dydt;
}

void Orbit::step(double dt) {
    auto f = [this](double t, const common::VectorX& y) {
        return this->dynamics(t, y);
    };

    state_ = engine::Integrator<common::VectorX>::rk4(0.0, state_, dt, f);
}

common::Vector3 Orbit::getPosition() const {
    return state_.segment<3>(0);
}

common::Vector3 Orbit::getVelocity() const {
    return state_.segment<3>(3);
}

common::Vector3 Orbit::computeJ2Acceleration(const common::Vector3& pos) const {
    // J2 perturbation acceleration (oblateness)
    // a_J2 = (3/2) * J2 * mu * R_E^2 / r^5 * f(x,y,z)
    // where f(x,y,z) accounts for directional effects
    
    double r = pos.norm();
    double r2 = r * r;
    double r5 = r2 * r2 * r;
    
    double x = pos.x();
    double y = pos.y();
    double z = pos.z();
    
    // Compute z/r ratio (used multiple times)
    double z_r = z / r;
    double z_r2 = z_r * z_r;
    
    // Common factor: (3/2) * J2 * mu * R_E^2 / r^5
    double factor = 1.5 * J2_EARTH * MU_EARTH * R_EARTH * R_EARTH / r5;
    
    // J2 acceleration components (using vector operations)
    // a_x = factor * x * (5*z_r^2 - 1)
    // a_y = factor * y * (5*z_r^2 - 1)
    // a_z = factor * z * (5*z_r^2 - 3)
    
    common::Vector3 acc_j2;
    double xy_factor = factor * (5.0 * z_r2 - 1.0);
    double z_factor = factor * (5.0 * z_r2 - 3.0);
    
    acc_j2.x() = xy_factor * x;
    acc_j2.y() = xy_factor * y;
    acc_j2.z() = z_factor * z;
    
    return acc_j2;
}

double Orbit::getSemiMajorAxis() const {
    common::Vector3 pos = getPosition();
    common::Vector3 vel = getVelocity();
    
    double r = pos.norm();
    double v2 = vel.squaredNorm();
    
    // Specific orbital energy: epsilon = v^2/2 - mu/r
    double energy = 0.5 * v2 - MU_EARTH / r;
    
    // Semi-major axis: a = -mu/(2*epsilon)
    return -MU_EARTH / (2.0 * energy);
}

double Orbit::getEccentricity() const {
    common::Vector3 pos = getPosition();
    common::Vector3 vel = getVelocity();
    
    double r = pos.norm();
    double v2 = vel.squaredNorm();
    
    // Specific angular momentum: h = r × v
    common::Vector3 h = pos.cross(vel);
    double h2 = h.squaredNorm();
    
    // Eccentricity vector: e = (v × h)/mu - r/|r|
    common::Vector3 e_vec = (vel.cross(h)) / MU_EARTH - pos / r;
    
    return e_vec.norm();
}

double Orbit::getInclination() const {
    common::Vector3 pos = getPosition();
    common::Vector3 vel = getVelocity();
    
    // Specific angular momentum: h = r × v
    common::Vector3 h = pos.cross(vel);
    
    // Inclination: i = acos(h_z / |h|)
    return std::acos(h.z() / h.norm());
}

double Orbit::getRaan() const {
    common::Vector3 pos = getPosition();
    common::Vector3 vel = getVelocity();
    
    // Specific angular momentum: h = r × v
    common::Vector3 h = pos.cross(vel);
    
    // Node vector: n = k × h (where k = [0, 0, 1])
    common::Vector3 k(0, 0, 1);
    common::Vector3 n = k.cross(h);
    
    if (n.norm() < 1e-10) {
        // Equatorial orbit, RAAN is undefined
        return 0.0;
    }
    
    // RAAN: raan = atan2(n_y, n_x)
    return std::atan2(n.y(), n.x());
}

double Orbit::getArgumentOfPerigee() const {
    common::Vector3 pos = getPosition();
    common::Vector3 vel = getVelocity();
    
    double r = pos.norm();
    
    // Specific angular momentum: h = r × v
    common::Vector3 h = pos.cross(vel);
    
    // Node vector: n = k × h
    common::Vector3 k(0, 0, 1);
    common::Vector3 n = k.cross(h);
    
    // Eccentricity vector: e = (v × h)/mu - r/|r|
    common::Vector3 e_vec = (vel.cross(h)) / MU_EARTH - pos / r;
    
    double e = e_vec.norm();
    if (e < 1e-10) {
        // Circular orbit, argument of perigee is undefined
        return 0.0;
    }
    
    if (n.norm() < 1e-10) {
        // Equatorial orbit, use different definition
        return std::atan2(e_vec.y(), e_vec.x());
    }
    
    // Argument of perigee: omega = acos(n · e / (|n| * |e|))
    double cos_omega = n.dot(e_vec) / (n.norm() * e);
    double omega = std::acos(std::clamp(cos_omega, -1.0, 1.0));
    
    // Check quadrant
    if (e_vec.z() < 0) {
        omega = 2.0 * M_PI - omega;
    }
    
    return omega;
}

} // namespace dynamics
} // namespace sim
