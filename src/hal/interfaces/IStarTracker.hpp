#pragma once
#include "common/types.hpp"
#include "hal/interfaces/ISensor.hpp"

namespace hal {

/**
 * @brief Interface for a Star Tracker sensor.
 *
 * Provides orientation measurement (Inertial -> Body).
 */
class IStarTracker : public ISensor {
   public:
    virtual ~IStarTracker() = default;

    std::string getName() const override {
        return "StarTracker";
    }

    /**
     * @brief Get the measured attitude quaternion (Inertial -> Body).
     *
     * @return common::Quaternion
     */
    virtual common::Quaternion getOrientation() const = 0;
};

}  // namespace hal
