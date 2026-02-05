#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>

namespace common {

class Logger {
public:
    static void Init() {
        auto console = spdlog::stdout_color_mt("console");
        spdlog::set_default_logger(console);
        spdlog::set_pattern("[%^%l%$] %v");
    }
};

// Simple wrappers for now
template<typename... Args>
inline void LogInfo(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    spdlog::info(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void LogError(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    spdlog::error(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void LogWarning(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    spdlog::warn(fmt, std::forward<Args>(args)...);
}

template<typename... Args>
inline void LogDebug(spdlog::format_string_t<Args...> fmt, Args &&...args) {
    spdlog::debug(fmt, std::forward<Args>(args)...);
}

} // namespace common
