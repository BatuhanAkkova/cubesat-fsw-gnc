#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common/logger.hpp"
#include "common/time.hpp"

namespace fsw {
namespace core {

/**
 * @brief Task definition for scheduler
 */
struct Task {
    std::string name;
    std::function<void(double)> callback;  // Callback receives dt
    double period_sec;                     // Execution period in seconds (e.g., 0.1 for 10Hz)
    double last_execution_time;            // Last execution timestamp
    int priority;                          // Higher value = higher priority

    Task(const std::string& n, std::function<void(double)> cb, double period, int prio = 0)
        : name(n), callback(cb), period_sec(period), last_execution_time(-period), priority(prio) {}
};

/**
 * @brief Statistics for scheduler performance
 */
struct SchedulerStats {
    double avg_jitter_ms;
    double max_jitter_ms;
    int cycles_executed;
    int task_overruns;

    SchedulerStats() : avg_jitter_ms(0.0), max_jitter_ms(0.0), cycles_executed(0), task_overruns(0) {}
};

/**
 * @brief Deterministic task scheduler for real-time GNC loops
 *
 * Executes registered tasks at fixed rates. Supports:
 * - Multiple tasks at different frequencies
 * - Priority-based execution
 * - Jitter monitoring
 * - Simulation time vs. wall-clock time
 */
class TaskScheduler {
   public:
    /**
     * @brief Constructor
     * @param base_dt Base time step for scheduler [seconds]
     * @param use_real_time If true, sleep to match real-time. If false, run as fast as possible (simulation)
     */
    explicit TaskScheduler(double base_dt = 0.1, bool use_real_time = false);

    /**
     * @brief Register a task
     * @param task Task to register
     */
    void registerTask(const Task& task);

    /**
     * @brief Start the scheduler (blocking call)
     * @param duration_sec Duration to run [seconds]. 0 = run indefinitely
     */
    void run(double duration_sec = 0.0);

    /**
     * @brief Execute a single step (for manual stepping in tests)
     * @param dt Time step [seconds]
     */
    void step(double dt);

    /**
     * @brief Stop the scheduler
     */
    void stop();

    /**
     * @brief Get current simulation time
     */
    double getCurrentTime() const {
        return current_time_;
    }

    /**
     * @brief Get scheduler statistics
     */
    SchedulerStats getStats() const {
        return stats_;
    }

    /**
     * @brief Reset scheduler (clear tasks and time)
     */
    void reset();

    /**
     * @brief Clear all registered tasks
     */
    void clearTasks();

   private:
    /**
     * @brief Execute all tasks that are due
     */
    void executeDueTasks(double dt);

    /**
     * @brief Update jitter statistics
     */
    void updateStats(double actual_dt);

    std::vector<Task> tasks_;
    double base_dt_;
    bool use_real_time_;
    double current_time_;
    bool running_;

    SchedulerStats stats_;
    std::chrono::high_resolution_clock::time_point last_wall_time_;

    std::mutex mutex_;
};

}  // namespace core
}  // namespace fsw
