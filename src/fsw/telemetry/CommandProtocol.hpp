#pragma once

#include <cstdint>

namespace fsw {
namespace telemetry {

/**
 * @brief Command APIDs
 */
enum class CommandAPID : uint16_t {
    SYSTEM  = 200,
    GNC     = 201,
    TEST    = 202
};

/**
 * @brief Command Function Codes for GNC (APID 201)
 */
enum class GNCFunctionCode : uint8_t {
    SLEW_TO_NADIR = 0x01,
    SLEW_TO_SUN   = 0x02,
    SET_GAINS     = 0x03,
    POINT_TARGET  = 0x04
};

/**
 * @brief Command Function Codes for SYSTEM (APID 200)
 */
enum class SystemFunctionCode : uint8_t {
    RESET       = 0x01,
    SET_MODE    = 0x02,
    TIME_SYNC   = 0x03
};

} // namespace telemetry
} // namespace fsw
