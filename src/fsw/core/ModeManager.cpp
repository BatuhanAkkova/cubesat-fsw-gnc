#include "fsw/core/ModeManager.hpp"

namespace fsw {
namespace core {

ModeManager::ModeManager(const ModeTransitionConfig& config)
    : current_mode_(MissionMode::SAFE),
      previous_mode_(MissionMode::SAFE),
      config_(config),
      time_in_current_mode_(0.0),
      mode_change_callback_(nullptr) {
    
    common::LogInfo("ModeManager initialized in SAFE mode");
}

void ModeManager::update(const common::Vector3& angular_velocity, double dt) {
    // Accumulate time in current mode
    time_in_current_mode_ += dt;
    
    // Evaluate potential mode transitions
    evaluateTransitions(angular_velocity);
}

void ModeManager::evaluateTransitions(const common::Vector3& angular_velocity) {
    double rate_norm = angular_velocity.norm();
    
    // Check if minimum time in mode has elapsed
    if (!canTransition()) {
        return;
    }
    
    MissionMode target_mode = current_mode_;
    
    switch (current_mode_) {
        case MissionMode::SAFE:
            // Transition to NOMINAL if rates are low enough
            if (rate_norm < config_.safe_to_nominal_rate_threshold) {
                target_mode = MissionMode::NOMINAL;
                common::LogInfo("Detumble complete (rate = {:.4f} rad/s), transitioning to NOMINAL", 
                                rate_norm);
            }
            break;
            
        case MissionMode::NOMINAL:
        case MissionMode::SCIENCE:
        case MissionMode::DOWNLINK:
        case MissionMode::DEGRADED:
            // Return to SAFE if rates are too high
            if (rate_norm > config_.nominal_to_safe_rate_threshold) {
                target_mode = MissionMode::SAFE;
                common::LogWarning("High angular rates detected ({:.4f} rad/s), transitioning to SAFE fallback", 
                                   rate_norm);
            }
            break;
            
        case MissionMode::CONTINGENCY:
            // Future: implement recovery logic
            break;
    }
    
    // Execute transition if needed
    if (target_mode != current_mode_) {
        transitionToMode(target_mode);
    }
}

void ModeManager::transitionToMode(MissionMode new_mode) {
    if (new_mode == current_mode_) {
        return;
    }
    
    previous_mode_ = current_mode_;
    current_mode_ = new_mode;
    time_in_current_mode_ = 0.0;
    
    common::LogInfo("Mode transition: {} -> {}", 
                    getModeString(previous_mode_), 
                    getModeString(current_mode_));
    
    // Invoke callback if registered
    if (mode_change_callback_) {
        mode_change_callback_(previous_mode_, current_mode_);
    }
}

bool ModeManager::commandMode(MissionMode new_mode) {
    if (new_mode == current_mode_) {
        return true;
    }
    
    common::LogInfo("Manual mode command: {} -> {}", 
                    getModeString(current_mode_), 
                    getModeString(new_mode));
    
    transitionToMode(new_mode);
    return true;
}

bool ModeManager::canTransition() const {
    return time_in_current_mode_ >= config_.min_time_in_mode;
}

void ModeManager::reset() {
    common::LogInfo("ModeManager reset to SAFE mode");
    transitionToMode(MissionMode::SAFE);
}

void ModeManager::forceModeChange(MissionMode new_mode, const std::string& reason) {
    if (new_mode == current_mode_) {
        common::LogWarning("FDIR force mode change ignored - already in {} mode: {}", 
                          getModeString(new_mode), reason);
        return;
    }
    
    common::LogWarning("FDIR forcing mode change: {} -> {} - Reason: {}", 
                      getModeString(current_mode_),
                      getModeString(new_mode),
                      reason);
    
    // Bypass timing constraint - go directly to transition
    transitionToMode(new_mode);
}


std::string ModeManager::getModeString(MissionMode mode) {
    switch (mode) {
        case MissionMode::SAFE:       return "SAFE";
        case MissionMode::NOMINAL:    return "NOMINAL";
        case MissionMode::SCIENCE:    return "SCIENCE";
        case MissionMode::DOWNLINK:   return "DOWNLINK";
        case MissionMode::DEGRADED:   return "DEGRADED";
        case MissionMode::CONTINGENCY: return "CONTINGENCY";
        default:                      return "UNKNOWN";
    }
}

} // namespace core
} // namespace fsw
