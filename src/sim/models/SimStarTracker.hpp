#pragma once

#include <random>

#include "hal/interfaces/IStarTracker.hpp"
#include "sim/dynamics/RigidBody.hpp"

namespace sim {
namespace models {

class SimStarTracker : public hal::IStarTracker {
   public:
    /**
     * @brief Construct a new Sim Star Tracker object
     *
     * @param body Reference to RigidBody for ground truth
     * @param noise_std Standard deviation of noise per axis [rad] typically small for ST
     */
    SimStarTracker(const dynamics::RigidBody& body, double noise_std = 0.0);

    common::Quaternion getOrientation() const override;

   private:
    const dynamics::RigidBody& body_;
    double noise_std_;

    mutable std::mt19937 gen_;
    mutable std::normal_distribution<double> dist_;
};

}  // namespace models
}  // namespace sim
