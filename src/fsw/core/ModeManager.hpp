#pragma once

#include <functional>
#include <memory>

#include "common/logger.hpp"
#include "common/types.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/control/Bdot.hpp"

namespace fsw {
namespace core {

/**
 * @brief Mission operating modes
 */
enum class MissionMode {
    SAFE,        // B-Dot detumbling mode
    NOMINAL,     // Normal pointing/control mode
    SCIENCE,     // Payload operations / Target pointing
    DOWNLINK,    // Ground station communication
    DEGRADED,    // Operation with reduced sensor confidence
    CONTINGENCY  // Emergency fallback (future use)
};

/**
 * @brief Mode transition conditions
 */
struct ModeTransitionConfig {
    double safe_to_nominal_rate_threshold = 0.02;  // rad/s - enter nominal when below
    double nominal_to_safe_rate_threshold = 0.1;   // rad/s - return to safe when above
    double min_time_in_mode = 10.0;                // seconds - prevent rapid switching
};

/**
 * @brief Finite State Machine for managing mission modes
 *
 * Manages automatic transitions between:
 * - SAFE mode: Uses B-Dot for detumbling
 * - NOMINAL mode: Uses PID-based attitude control
 *
 * Transitions are based on angular rate thresholds and time constraints.
 */
class ModeManager {
   public:
    using ModeChangeCallback = std::function<void(MissionMode, MissionMode)>;

    /**
     * @brief Constructor
     * @param config Transition configuration parameters
     */
    explicit ModeManager(const ModeTransitionConfig& config = ModeTransitionConfig());

    /**
     * @brief Update the mode manager (call every control cycle)
     * @param angular_velocity Current spacecraft angular velocity [rad/s]
     * @param dt Time step [seconds]
     */
    void update(const common::Vector3& angular_velocity, double dt);

    /**
     * @brief Get current mission mode
     */
    MissionMode getCurrentMode() const {
        return current_mode_;
    }

    /**
     * @brief Manually command a mode change
     * @param new_mode Target mode
     * @return true if transition was successful
     */
    bool commandMode(MissionMode new_mode);

    /**
     * @brief Register callback for mode changes
     * @param callback Function to call when mode changes
     */
    void setModeChangeCallback(ModeChangeCallback callback) {
        mode_change_callback_ = callback;
    }

    /**
     * @brief Force mode change (bypasses timing constraints)
     *
     * Used by FDIR to transition immediately during critical failures.
     * Unlike commandMode(), this does not check minimum time in mode.
     *
     * @param new_mode Target mode
     * @param reason Reason for forced transition (logged)
     */
    void forceModeChange(MissionMode new_mode, const std::string& reason);

    /**
     * @brief Get time in current mode
     */
    double getTimeInMode() const {
        return time_in_current_mode_;
    }

    /**
     * @brief Reset the mode manager to SAFE mode
     */
    void reset();

    /**
     * @brief Get mode name as string (for logging)
     */
    static std::string getModeString(MissionMode mode);

   private:
    /**
     * @brief Evaluate and execute mode transitions
     */
    void evaluateTransitions(const common::Vector3& angular_velocity);

    /**
     * @brief Execute a mode transition
     */
    void transitionToMode(MissionMode new_mode);

    /**
     * @brief Check if transition is allowed based on timing
     */
    bool canTransition() const;

    MissionMode current_mode_;
    MissionMode previous_mode_;
    ModeTransitionConfig config_;

    double time_in_current_mode_;
    ModeChangeCallback mode_change_callback_;
};

}  // namespace core
}  // namespace fsw
