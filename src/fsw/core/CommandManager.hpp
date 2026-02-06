#pragma once

#include "fsw/core/Command.hpp"
#include <queue>
#include <mutex>
#include <memory>

namespace fsw {
namespace core {

/**
 * @brief Manager to handle receiving, queuing, and executing commands.
 */
class CommandManager {
public:
    /**
     * @brief Add a command to the execution queue.
     */
    void enqueueCommand(std::unique_ptr<Command> cmd);

    /**
     * @brief Process all queued commands.
     * Should be called periodically in the FSW loop.
     */
    void update();

    /**
     * @brief Get number of pending commands.
     */
    size_t getPendingCount();

private:
    std::queue<std::unique_ptr<Command>> command_queue_;
    std::mutex queue_mutex_;
};

} // namespace core
} // namespace fsw
