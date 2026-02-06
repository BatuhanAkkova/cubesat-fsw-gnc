#include "fsw/core/CommandManager.hpp"
#include "common/logger.hpp"

namespace fsw {
namespace core {

void CommandManager::enqueueCommand(std::unique_ptr<Command> cmd) {
    if (!cmd) return;
    
    std::lock_guard<std::mutex> lock(queue_mutex_);
    common::LogInfo("Enqueued command: {}", cmd->getName());
    command_queue_.push(std::move(cmd));
}

void CommandManager::update() {
    std::unique_ptr<Command> cmd;
    
    while (true) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (command_queue_.empty()) break;
            
            cmd = std::move(command_queue_.front());
            command_queue_.pop();
        }
        
        if (cmd) {
            common::LogInfo("Executing command: {}", cmd->getName());
            if (!cmd->execute()) {
                common::LogWarning("Command execution failed: {}", cmd->getName());
            }
        }
    }
}

size_t CommandManager::getPendingCount() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return command_queue_.size();
}

} // namespace core
} // namespace fsw
