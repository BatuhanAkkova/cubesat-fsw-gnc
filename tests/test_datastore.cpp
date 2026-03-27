#include <gtest/gtest.h>
#include <string>
#include <thread>

#include "fsw/core/DataStore.hpp"

using namespace fsw;

TEST(DataStoreTest, BasicPubSub) {
    auto& ds = DataStore::Instance();
    bool callback_received = false;
    int received_val = 0;

    ds.subscribe<int>("test_topic", [&](const int& val) {
        callback_received = true;
        received_val = val;
    });

    ds.publish("test_topic", 42);

    EXPECT_TRUE(callback_received);
    EXPECT_EQ(received_val, 42);
}

TEST(DataStoreTest, GetMethod) {
    auto& ds = DataStore::Instance();
    ds.publish("get_topic", 123.45);

    double val = 0.0;
    bool exists = ds.get("get_topic", val);

    EXPECT_TRUE(exists);
    EXPECT_DOUBLE_EQ(val, 123.45);

    int wrong_type;
    bool bad_cast = ds.get("get_topic", wrong_type);
    EXPECT_FALSE(bad_cast);
}

TEST(DataStoreTest, ThreadSafety) {
    auto& ds = DataStore::Instance();
    const int N = 100;
    std::vector<std::thread> threads;

    // Multiple publishers
    for (int i = 0; i < N; ++i) {
        threads.emplace_back([&ds, i]() { ds.publish("thread_topic", i); });
    }

    for (auto& t : threads) {
        t.join();
    }

    int val = 0;
    EXPECT_TRUE(ds.get("thread_topic", val));
}
