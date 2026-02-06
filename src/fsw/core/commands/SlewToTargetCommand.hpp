#pragma once

#include "fsw/core/Command.hpp"
#include "fsw/core/DataStore.hpp"
#include "common/types.hpp"
#include "common/logger.hpp"

namespace fsw {
namespace core {
namespace commands {

/**
 * @brief Command to change guidance mode to Target pointing and set the target quaternion.
 */
class SlewToTargetCommand : public Command {
public:
    explicit SlewToTargetCommand(const common::Quaternion& target_q) : target_q_(target_q) {}

    bool execute() override {
        common::LogInfo("Executing SlewToTargetCommand");
        // Publish target quaternion and notify guidance mode change
        fsw::DataStore::Instance().publish<common::Quaternion>("guidance/target_quaternion", target_q_);
        fsw::DataStore::Instance().publish<std::string>("guidance/target_mode", "TARGET");
        return true;
    }

    std::string getName() const override {
        return "SlewToTarget";
    }

private:
    common::Quaternion target_q_;
};

} // namespace commands
} // namespace core
} // namespace fsw
