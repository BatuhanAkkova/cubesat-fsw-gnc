#pragma once
#include "common/types.hpp"
#include "hal/interfaces/IActuator.hpp"

namespace hal {

/**
 * @brief Interface for a Magnetorquer actuator.
 */
class ITorquer : public IActuator {
public:
    virtual ~ITorquer() = default;

    std::string getName() const override { return "Torquer"; }

    /**
     * @brief Command a magnetic dipole moment.
     * @param dipole_moment_Am2 Desired dipole moment in [Am^2] in the body frame (or component frame).
     */
    virtual void setDipole(const common::Vector3& dipole_moment_Am2) = 0;
};

} // namespace hal
