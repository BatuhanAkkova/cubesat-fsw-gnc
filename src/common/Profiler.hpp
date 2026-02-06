#pragma once

#include <chrono>
#include <map>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <iomanip>

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
     * @brief Print all statistics to stdout.
     */
    void printReport() const {
        std::cout << "\n========== PROFILING REPORT ==========" << std::endl;
        if (stats_.empty()) {
            return;
        }
        std::cout << std::left << std::setw(20) << "Section" 
                  << std::right << std::setw(12) << "Avg(ms)" 
                  << std::setw(12) << "Max(ms)" 
                  << std::setw(12) << "Total(ms)" 
                  << std::setw(10) << "Calls" << std::endl;
        std::cout << std::string(66, '-') << std::endl;

        for (auto const& [name, s] : stats_) {
            std::cout << std::left << std::setw(20) << name 
                      << std::right << std::fixed << std::setprecision(4) 
                      << std::setw(12) << s.avg_time_ms() 
                      << std::setw(12) << s.max_time_ms 
                      << std::setw(12) << s.total_time_ms 
                      << std::setw(10) << s.count << std::endl;
        }
        std::cout << "======================================\n" << std::endl;
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
