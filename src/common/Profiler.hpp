#pragma once

#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include "common/logger.hpp"

namespace common {

/**
 * @brief Simple RAII profiler to measure execution time of code blocks.
 */
class Profiler {
public:
    struct Stats {
        double total_time_ms = 0.0;
        double max_time_ms = 0.0;
        uint64_t count = 0;
        
        double avg_time_ms() const {
            return count > 0 ? total_time_ms / count : 0.0;
        }
    };

    /**
     * @brief Start timing a specific section.
     */
    void start(const std::string& name) {
        start_times_[name] = std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Stop timing and record results.
     */
    void stop(const std::string& name) {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto it = start_times_.find(name);
        if (it != start_times_.end()) {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - it->second);
            double ms = duration.count() / 1000.0;
            
            Stats& s = stats_[name];
            s.total_time_ms += ms;
            s.count++;
            s.max_time_ms = std::max(s.max_time_ms, ms);
        }
    }

    /**
     * @brief Print all statistics to the log.
     */
    void printReport() const {
        if (stats_.empty()) {
            return;
        }

        common::LogInfo("\n========== PROFILING REPORT ==========");
        common::LogInfo("{:<20} {:>12} {:>12} {:>12} {:>10}", "Section", "Avg(ms)", "Max(ms)", "Total(ms)", "Calls");
        common::LogInfo("{}", std::string(66, '-'));

        for (auto const& [name, s] : stats_) {
            common::LogInfo("{:<20} {:>12.4f} {:>12.4f} {:>12.4f} {:>10}",
                            name, s.avg_time_ms(), s.max_time_ms, s.total_time_ms, s.count);
        }
        common::LogInfo("======================================\n");
    }

    /**
     * @brief Reset all statistics.
     */
    void reset() {
        stats_.clear();
        start_times_.clear();
    }

private:
    std::map<std::string, Stats> stats_;
    std::map<std::string, std::chrono::time_point<std::chrono::high_resolution_clock>> start_times_;
};

/**
 * @brief Scoped timer for easier profiling.
 */
class ScopedTimer {
public:
    ScopedTimer(Profiler& profiler, const std::string& name) 
        : profiler_(profiler), name_(name) {
        profiler_.start(name_);
    }
    
    ~ScopedTimer() {
        profiler_.stop(name_);
    }

private:
    Profiler& profiler_;
    std::string name_;
};

} // namespace common
