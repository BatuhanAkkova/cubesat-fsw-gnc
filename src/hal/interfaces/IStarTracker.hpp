#pragma once

#include "common/types.hpp"

namespace hal {

/**
 * @brief Interface for a Star Tracker sensor.
 * 
 * Provides orientation measurement (Inertial -> Body).
 */
class IStarTracker {
public:
    virtual ~IStarTracker() = default;

    /**
     * @brief Get the measured attitude quaternion (Inertial -> Body).
     * 
     * @return common::Quaternion 
     */
    virtual common::Quaternion getOrientation() const = 0;
};

} // namespace hal
