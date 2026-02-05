#pragma once
#include "hal/interfaces/ITorquer.hpp"
#include <algorithm>

namespace sim {
namespace models {

/**
 * @brief Simulated Magnetorquer
 */
class SimTorquer : public hal::ITorquer {
public:
    /**
     * @brief Constructor
     * @param max_dipole Maximum dipole moment [Am^2] per axis.
     */
    explicit SimTorquer(double max_dipole);

    // From ITorquer
    void setDipole(const common::Vector3& dipole_moment_Am2) override;

    /**
     * @brief Get the actual dipole moment produced by the torquer (includes saturation).
     * @return Dipole moment [Am^2] in Body frame
     */
    common::Vector3 getDipole() const;

private:
    double max_dipole_;
    common::Vector3 current_dipole_;
};

} // namespace models
} // namespace sim
