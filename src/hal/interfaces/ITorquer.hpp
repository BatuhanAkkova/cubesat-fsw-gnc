#pragma once
#include "common/types.hpp"

namespace hal {

/**
 * @brief Interface for a Magnetorquer actuator.
 */
class ITorquer {
public:
    virtual ~ITorquer() = default;

    /**
     * @brief Command a magnetic dipole moment.
     * @param dipole_moment_Am2 Desired dipole moment in [Am^2] in the body frame (or component frame).
     */
    virtual void setDipole(const common::Vector3& dipole_moment_Am2) = 0;
};

} // namespace hal
