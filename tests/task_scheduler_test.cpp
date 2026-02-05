#include <gtest/gtest.h>
#include "fsw/core/TaskScheduler.hpp"
#include <atomic>
#include <iostream>

using namespace fsw::core;

class TaskSchedulerTest : public ::testing::Test {
protected:
    void SetUp() override {
        execution_count_10hz = 0;
        execution_count_1hz = 0;
    }

    std::atomic<int> execution_count_10hz;
    std::atomic<int> execution_count_1hz;
};

TEST_F(TaskSchedulerTest, SingleTaskExecution) {
    TaskScheduler scheduler(0.1, false);  // 10Hz base rate, simulation mode
    
    int count = 0;
    Task task("test_task", [&](double dt) {
        count++;
    }, 0.1);
    
    scheduler.registerTask(task);
    
    // Execute 10 steps
    for (int i = 0; i < 10; ++i) {
        scheduler.step(0.1);
    }
    
    // Task should have executed 10 times (once per step)
    EXPECT_EQ(count, 10);
    EXPECT_DOUBLE_EQ(scheduler.getCurrentTime(), 1.0);
}

TEST_F(TaskSchedulerTest, MultiRateTasks) {
    TaskScheduler scheduler(0.1, false);
    
    // 10Hz task
    Task task_10hz("task_10hz", [&](double dt) {
        execution_count_10hz++;
    }, 0.1);
    
    // 1Hz task
    Task task_1hz("task_1hz", [&](double dt) {
        execution_count_1hz++;
    }, 1.0);
    
    scheduler.registerTask(task_10hz);
    scheduler.registerTask(task_1hz);
    
    // Run for 2 seconds (20 steps at 0.1s each)
    for (int i = 0; i < 20; ++i) {
        scheduler.step(0.1);
    }
    
    // 10Hz task should execute 20 times
    EXPECT_EQ(execution_count_10hz.load(), 20);
    
    // 1Hz task should execute 2 times
    EXPECT_EQ(execution_count_1hz.load(), 2);
}

TEST_F(TaskSchedulerTest, TaskPriority) {
    TaskScheduler scheduler(0.1, false);
    
    std::vector<std::string> execution_order;
    
    // Create tasks with different priorities
    Task low_priority("low", [&](double dt) {
        execution_order.push_back("low");
    }, 0.1, 1);
    
    Task high_priority("high", [&](double dt) {
        execution_order.push_back("high");
    }, 0.1, 10);
    
    Task medium_priority("medium", [&](double dt) {
        execution_order.push_back("medium");
    }, 0.1, 5);
    
    // Register in random order
    scheduler.registerTask(low_priority);
    scheduler.registerTask(high_priority);
    scheduler.registerTask(medium_priority);
    
    // Execute one step
    scheduler.step(0.1);
    
    // Should execute in priority order: high, medium, low
    ASSERT_EQ(execution_order.size(), 3);
    EXPECT_EQ(execution_order[0], "high");
    EXPECT_EQ(execution_order[1], "medium");
    EXPECT_EQ(execution_order[2], "low");
}

TEST_F(TaskSchedulerTest, RunWithDuration) {
    TaskScheduler scheduler(0.1, false);
    
    int count = 0;
    Task task("test_task", [&](double dt) {
        count++;
    }, 0.1);
    
    scheduler.registerTask(task);
    
    // Run for 1.0 second
    scheduler.run(1.0);
    
    // Should have executed approximately 10 times
    EXPECT_GE(count, 9);
    EXPECT_LE(count, 11);
    EXPECT_GE(scheduler.getCurrentTime(), 1.0);
}

TEST_F(TaskSchedulerTest, TaskException) {
    TaskScheduler scheduler(0.1, false);
    
    int count = 0;
    Task good_task("good", [&](double dt) {
        count++;
    }, 0.1);
    
    Task bad_task("bad", [&](double dt) {
        throw std::runtime_error("Test exception");
    }, 0.1);
    
    scheduler.registerTask(good_task);
    scheduler.registerTask(bad_task);
    
    // Should not crash, good task should still execute
    scheduler.step(0.1);
    scheduler.step(0.1);
    
    EXPECT_EQ(count, 2);  // Good task executed twice
}

TEST_F(TaskSchedulerTest, Reset) {
    TaskScheduler scheduler(0.1, false);
    
    int count = 0;
    Task task("test_task", [&](double dt) {
        count++;
    }, 0.1);
    
    scheduler.registerTask(task);
    
    for (int i = 0; i < 5; ++i) {
        scheduler.step(0.1);
    }
    
    EXPECT_EQ(count, 5);
    
    // Reset
    scheduler.reset();
    
    EXPECT_EQ(scheduler.getCurrentTime(), 0.0);
    
    // Step after reset - task should not execute (was cleared)
    scheduler.step(0.1);
    EXPECT_EQ(count, 5);  // Still 5, no new execution
}

TEST_F(TaskSchedulerTest, Statistics) {
    TaskScheduler scheduler(0.1, false);
    
    Task task("test", [](double dt) {}, 0.1);
    scheduler.registerTask(task);
    
    for (int i = 0; i < 10; ++i) {
        scheduler.step(0.1);
    }
    
    SchedulerStats stats = scheduler.getStats();
    EXPECT_EQ(stats.cycles_executed, 10);
    // In simulation mode (non-real-time), jitter is not tracked
}

TEST_F(TaskSchedulerTest, DifferentPeriodTask) {
    TaskScheduler scheduler(0.05, false);  // 20Hz base
    
    int count_20hz = 0;
    int count_5hz = 0;
    
    Task task_20hz("20hz", [&](double dt) {
        count_20hz++;
    }, 0.05);  // 20Hz
    
    Task task_5hz("5hz", [&](double dt) {
        count_5hz++;
    }, 0.2);  // 5Hz
    
    scheduler.registerTask(task_20hz);
    scheduler.registerTask(task_5hz);
    
    // Run for 1 second (20 steps at 0.05s)
    for (int i = 0; i < 20; ++i) {
        scheduler.step(0.05);
    }
    
    EXPECT_EQ(count_20hz, 20);  // 20Hz * 1s = 20
    EXPECT_EQ(count_5hz, 5);    // 5Hz * 1s = 5
}
