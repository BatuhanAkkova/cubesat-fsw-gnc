#include "sim/models/SimTorquer.hpp"

namespace sim {
namespace models {

SimTorquer::SimTorquer(double max_dipole) : max_dipole_(max_dipole), current_dipole_(common::Vector3::Zero()) {}

void SimTorquer::setDipole(const common::Vector3& dipole_moment_Am2) {
    // Saturate the command
    current_dipole_.x() = std::max(-max_dipole_, std::min(max_dipole_, dipole_moment_Am2.x()));
    current_dipole_.y() = std::max(-max_dipole_, std::min(max_dipole_, dipole_moment_Am2.y()));
    current_dipole_.z() = std::max(-max_dipole_, std::min(max_dipole_, dipole_moment_Am2.z()));
}

common::Vector3 SimTorquer::getDipole() const {
    return current_dipole_;
}

}  // namespace models
}  // namespace sim
