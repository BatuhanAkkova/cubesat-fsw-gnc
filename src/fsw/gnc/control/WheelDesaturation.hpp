#pragma once

#include <memory>
#include <vector>

#include "common/types.hpp"
#include "hal/interfaces/IRW.hpp"
#include "hal/interfaces/ITorquer.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief Wheel Desaturation Controller
 *
 * Manages reaction wheel momentum and commands magnetorquers to desaturate
 * wheels using B-cross-H control law.
 *
 * Algorithm: M_cmd = -k * (h_wheels × B) × B / |B|^2
 */
class WheelDesaturation {
   public:
    /**
     * @brief Configuration parameters
     */
    struct Config {
        double momentum_threshold_ratio;  // Trigger at 80% of max momentum
        double desat_gain;                // Control gain for B-cross-H [A*m^2/(Nms * T)]
        double max_desat_duration;        // Maximum time in desaturation [s]
        double min_time_between_desat;    // Minimum cooldown period [s]
        double min_b_field_magnitude;     // Minimum B-field to attempt desaturation [T]

        Config()
            : momentum_threshold_ratio(0.8),
              desat_gain(1e-6),
              max_desat_duration(300.0),
              min_time_between_desat(60.0),
              min_b_field_magnitude(1e-6) {}
    };

    /**
     * @brief Desaturation state machine states
     */
    enum class State {
        IDLE,          // Not desaturating
        DESATURATING,  // Actively desaturating
        COOLDOWN       // Waiting before next desaturation
    };

    /**
     * @brief Constructor
     * @param config Configuration parameters
     */
    explicit WheelDesaturation(const Config& config = Config());

    /**
     * @brief Update desaturation controller
     *
     * @param wheels Vector of reaction wheel pointers
     * @param b_field_body Measured B-field in body frame [T]
     * @param dt Time step [s]
     * @return Commanded magnetorquer dipole [A*m^2]
     */
    common::Vector3 update(const std::vector<std::shared_ptr<hal::IRW>>& wheels, const common::Vector3& b_field_body,
                           double dt);

    /**
     * @brief Get current state
     */
    State getState() const {
        return state_;
    }

    /**
     * @brief Get current total angular momentum
     * @return Total wheel momentum magnitude [Nms]
     */
    double getTotalMomentum() const {
        return total_momentum_;
    }

    /**
     * @brief Get maximum allowable momentum
     * @return Max momentum [Nms]
     */
    double getMaxMomentum() const {
        return max_momentum_;
    }

    /**
     * @brief Force reset to IDLE state
     */
    void reset();

   private:
    Config config_;
    State state_;
    double max_momentum_;    // Maximum total momentum [Nms]
    double total_momentum_;  // Current total momentum [Nms]
    double time_in_state_;   // Time spent in current state [s]

    /**
     * @brief Compute total angular momentum from all wheels
     */
    common::Vector3 computeTotalMomentum(const std::vector<std::shared_ptr<hal::IRW>>& wheels) const;

    /**
     * @brief Compute maximum total momentum from all wheels
     */
    double computeMaxMomentum(const std::vector<std::shared_ptr<hal::IRW>>& wheels) const;

    /**
     * @brief Update state machine
     */
    void updateStateMachine(double dt);

    /**
     * @brief Compute B-cross-H control law
     * @param h_wheels Angular momentum vector [Nms]
     * @param b_field B-field vector [T]
     * @return Commanded dipole moment [A*m^2]
     */
    common::Vector3 computeBCrossH(const common::Vector3& h_wheels, const common::Vector3& b_field) const;
};

}  // namespace control
}  // namespace gnc
}  // namespace fsw
