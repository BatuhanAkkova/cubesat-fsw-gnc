#pragma once

#include "common/logger.hpp"
#include "fsw/core/Command.hpp"
#include "fsw/core/DataStore.hpp"
#include "fsw/gnc/control/AttitudeController.hpp"

namespace fsw {
namespace core {
namespace commands {

/**
 * @brief Command to update PID gains in the AttitudeController.
 */
using fsw::gnc::control::GainsPayload;

class SetPidGainsCommand : public Command {
   public:
    SetPidGainsCommand(double kp, double ki, double kd, bool is_nominal)
        : kp_(kp), ki_(ki), kd_(kd), is_nominal_(is_nominal) {}

    bool execute() override {
        common::LogInfo("Executing SetPidGainsCommand (kp={}, ki={}, kd={}, nominal={})", kp_, ki_, kd_, is_nominal_);

        GainsPayload payload{kp_, ki_, kd_, is_nominal_};
        fsw::DataStore::Instance().publish<GainsPayload>("gnc/att_gains", payload);

        return true;
    }

    std::string getName() const override {
        return "SetPidGains";
    }

   private:
    double kp_, ki_, kd_;
    bool is_nominal_;
};

}  // namespace commands
}  // namespace core
}  // namespace fsw
