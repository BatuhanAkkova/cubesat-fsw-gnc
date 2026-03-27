#pragma once

#include "common/logger.hpp"
#include "fsw/core/Command.hpp"
#include "fsw/core/DataStore.hpp"

namespace fsw {
namespace core {
namespace commands {

/**
 * @brief Command to change guidance mode to Nadir pointing.
 */
class SlewToNadirCommand : public Command {
   public:
    bool execute() override {
        common::LogInfo("Executing SlewToNadirCommand");
        // Publish to DataStore to notify guidance/FlightSoftware
        fsw::DataStore::Instance().publish<std::string>("guidance/target_mode", "NADIR");
        return true;
    }

    std::string getName() const override {
        return "SlewToNadir";
    }
};

}  // namespace commands
}  // namespace core
}  // namespace fsw
