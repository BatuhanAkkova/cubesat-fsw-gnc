#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fsw {
namespace core {

/**
 * @brief Abstract base class for all FSW commands.
 */
class Command {
   public:
    virtual ~Command() = default;

    /**
     * @brief Execute the command logic.
     * @return true if execution was successful or initiated.
     */
    virtual bool execute() = 0;

    /**
     * @brief Get name of the command for logging.
     */
    virtual std::string getName() const = 0;
};

}  // namespace core
}  // namespace fsw
