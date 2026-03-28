#include <chrono>
#include <gtest/gtest.h>
#include <numeric>
#include <thread>
#include <vector>

#include "fsw/core/SPSCQueue.hpp"

#include "common/StateHistory.hpp"

using namespace fsw::core;
using namespace common;

// Test SPSCQueue correctness and concurrency
TEST(PerformanceTest, SPSCQueueConcurrency) {
    SPSCQueue<int, 1024> queue;
    const int num_elements = 10000;
    std::vector<int> received;

    std::thread producer([&]() {
        for (int i = 0; i < num_elements; ++i) {
            while (!queue.push(i)) {
                std::this_thread::yield();
            }
        }
    });

    std::thread consumer([&]() {
        int val;
        for (int i = 0; i < num_elements; ++i) {
            while (!queue.pop(val)) {
                std::this_thread::yield();
            }
            received.push_back(val);
        }
    });

    producer.join();
    consumer.join();

    EXPECT_EQ(received.size(), num_elements);
    for (int i = 0; i < num_elements; ++i) {
        EXPECT_EQ(received[i], i);
    }
}

// Benchmark SPSCQueue vs Mutex (conceptual)
TEST(PerformanceTest, SPSCQueueLatency) {
    SPSCQueue<int, 1024> queue;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        queue.push(i);
        int val;
        queue.pop(val);
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "[ INFO ] SPSCQueue average push-pop latency: " << duration / 10000.0 << " ns" << std::endl;
}

// Test StateHistory SIMD Performance
TEST(PerformanceTest, StateHistorySIMD) {
    const size_t n_samples = 10000;
    StateHistory history(n_samples);

    // Fill with dummy data
    for (size_t i = 0; i < n_samples; ++i) {
        State s;
        s.q = Quaternion::Identity();
        s.w = Vector3(1.0, 2.0, 3.0);
        s.pos = Vector3(i, i, i);
        s.vel = Vector3(0.1, 0.1, 0.1);
        history.addState(s);
    }

    // Benchmark Mean calculation (SIMD-optimized)
    auto start = std::chrono::high_resolution_clock::now();
    double m = history.mean(StateHistory::POSX);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

    EXPECT_NEAR(m, (n_samples - 1) / 2.0, 1e-7);
    std::cout << "[ INFO ] StateHistory SIMD mean calculation for " << n_samples << " samples: " << duration << " ns"
              << std::endl;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
