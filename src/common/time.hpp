#pragma once

#include <chrono>
#include <cstdint>

namespace common {

/**
 * @brief Represents spacecraft elapsed time in seconds.
 *
 * Uses double precision for simplicity in simulation and GNC math.
 * In a real flight computer, implement rollover or tick counts.
 */
class SpacecraftTime {
   public:
    SpacecraftTime() : seconds_(0.0) {}
    explicit SpacecraftTime(double seconds) : seconds_(seconds) {}

    double as_seconds() const {
        return seconds_;
    }

    // Operators
    SpacecraftTime operator+(const double& dt) const {
        return SpacecraftTime(seconds_ + dt);
    }

    void operator+=(const double& dt) {
        seconds_ += dt;
    }

    bool operator>(const SpacecraftTime& other) const {
        return seconds_ > other.seconds_;
    }

    bool operator<(const SpacecraftTime& other) const {
        return seconds_ < other.seconds_;
    }

   private:
    double seconds_;
};

}  // namespace common
