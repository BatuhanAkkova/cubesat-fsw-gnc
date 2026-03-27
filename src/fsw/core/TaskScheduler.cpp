#include "fsw/core/TaskScheduler.hpp"

#include <algorithm>
#include <cmath>

namespace fsw {
namespace core {

TaskScheduler::TaskScheduler(double base_dt, bool use_real_time)
    : base_dt_(base_dt), use_real_time_(use_real_time), current_time_(0.0), running_(false), stats_() {
    common::LogInfo("TaskScheduler initialized (dt={:.3f}s, real_time={})", base_dt_, use_real_time_);
}

void TaskScheduler::registerTask(const Task& task) {
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_.push_back(task);

    // Sort by priority (highest first)
    std::sort(tasks_.begin(), tasks_.end(), [](const Task& a, const Task& b) { return a.priority > b.priority; });

    common::LogInfo("Registered task '{}' at {:.1f}Hz (period={:.3f}s, priority={})", task.name, 1.0 / task.period_sec,
                    task.period_sec, task.priority);
}

void TaskScheduler::run(double duration_sec) {
    running_ = true;
    current_time_ = 0.0;
    stats_ = SchedulerStats();

    last_wall_time_ = std::chrono::high_resolution_clock::now();

    common::LogInfo("TaskScheduler started (duration={:.1f}s)", duration_sec);

    while (running_) {
        auto step_start = std::chrono::high_resolution_clock::now();

        // Execute tasks due in this cycle
        step(base_dt_);

        // Check if duration reached
        if (duration_sec > 0.0 && current_time_ >= duration_sec) {
            common::LogInfo("TaskScheduler completed {:.1f}s duration", duration_sec);
            break;
        }

        // Sleep to maintain real-time rate (if enabled)
        if (use_real_time_) {
            auto step_end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration<double>(step_end - step_start).count();
            double sleep_time = base_dt_ - elapsed;

            if (sleep_time > 0.0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
            } else {
                stats_.task_overruns++;
                common::LogWarning("Task overrun: cycle took {:.3f}ms (target {:.3f}ms)", elapsed * 1000.0,
                                   base_dt_ * 1000.0);
            }
        }

        // Update statistics
        auto cycle_end = std::chrono::high_resolution_clock::now();
        double actual_dt = std::chrono::duration<double>(cycle_end - last_wall_time_).count();
        updateStats(actual_dt);
        last_wall_time_ = cycle_end;
    }

    common::LogInfo(
        "TaskScheduler stopped at t={:.2f}s (cycles={}, avg_jitter={:.2f}ms, max_jitter={:.2f}ms, overruns={})",
        current_time_, stats_.cycles_executed, stats_.avg_jitter_ms, stats_.max_jitter_ms, stats_.task_overruns);
}

void TaskScheduler::step(double dt) {
    std::lock_guard<std::mutex> lock(mutex_);

    executeDueTasks(dt);
    current_time_ += dt;
    stats_.cycles_executed++;
}

void TaskScheduler::executeDueTasks(double dt) {
    for (auto& task : tasks_) {
        double time_since_last = current_time_ - task.last_execution_time;

        // Check if task is due (or if it's the first execution)
        if (time_since_last >= task.period_sec - 1e-6) {  // Small epsilon for floating point
            try {
                task.callback(dt);
                task.last_execution_time = current_time_;
            } catch (const std::exception& e) {
                common::LogError("Task '{}' threw exception: {}", task.name, e.what());
            }
        }
    }
}

void TaskScheduler::updateStats(double actual_dt) {
    if (use_real_time_ && stats_.cycles_executed > 0) {
        double jitter_ms = std::abs(actual_dt - base_dt_) * 1000.0;

        // Update max jitter
        stats_.max_jitter_ms = std::max(stats_.max_jitter_ms, jitter_ms);

        // Update average jitter (running average)
        double alpha = 0.1;  // Smoothing factor
        stats_.avg_jitter_ms = alpha * jitter_ms + (1.0 - alpha) * stats_.avg_jitter_ms;
    }
}

void TaskScheduler::stop() {
    running_ = false;
    common::LogInfo("TaskScheduler stop requested");
}

void TaskScheduler::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_.clear();
    current_time_ = 0.0;
    running_ = false;
    stats_ = SchedulerStats();
    common::LogInfo("TaskScheduler reset");
}

void TaskScheduler::clearTasks() {
    std::lock_guard<std::mutex> lock(mutex_);
    tasks_.clear();
    common::LogInfo("TaskScheduler tasks cleared");
}

}  // namespace core
}  // namespace fsw
