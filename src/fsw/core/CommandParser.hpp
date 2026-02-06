#pragma once

#include "fsw/telemetry/CCSDS.hpp"
#include "fsw/core/Command.hpp"
#include <memory>
#include <vector>

namespace fsw {
namespace core {

/**
 * @brief Parser to convert CCSDS packets into Command objects.
 */
class CommandParser {
public:
    /**
     * @brief Parse a raw packet into a Command object.
     * @param raw_packet Raw byte vector of the CCSDS packet.
     * @return unique_ptr to the command, or nullptr if parsing fails.
     */
    static std::unique_ptr<Command> parse(const std::vector<uint8_t>& raw_packet);
};

} // namespace core
} // namespace fsw
