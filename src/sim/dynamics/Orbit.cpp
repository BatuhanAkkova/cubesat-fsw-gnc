#include "Orbit.hpp"
#include "sim/engine/Integrator.hpp"
#include <cmath>

namespace sim {
namespace dynamics {

// Standard gravitational parameter for Earth [m^3/s^2]
static constexpr double MU_EARTH = 3.986004418e14;

Orbit::Orbit(const common::Vector3& init_pos, const common::Vector3& init_vel) {
    state_.resize(6);
    state_.segment<3>(0) = init_pos;
    state_.segment<3>(3) = init_vel;
}

common::VectorX Orbit::dynamics(double, const common::VectorX& y) {
    common::Vector3 pos = y.segment<3>(0);
    common::Vector3 vel = y.segment<3>(3);

    double r_norm = pos.norm();
    double mu_r3 = MU_EARTH / (std::pow(r_norm, 3));

    common::Vector3 acc = -mu_r3 * pos;

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

} // namespace dynamics
} // namespace sim
