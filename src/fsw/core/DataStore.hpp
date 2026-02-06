#pragma once

#include <string>
#include <unordered_map>
#include <functional>
#include <vector>
#include <mutex>
#include <memory>
#include <any>
#include <iostream>

#include "common/logger.hpp"

namespace fsw {

/**
 * @brief Thread-safe DataStore for Pub/Sub.
 * 
 * Allows modules to subscribe to data topics and publish updates.
 * Simplified implementation using std::any for type erasure (C++17).
 */
class DataStore {
public:
    using Callback = std::function<void(const std::any&)>;

    static DataStore& Instance() {
        static DataStore instance;
        return instance;
    }

    // Prevent copies
    DataStore(const DataStore&) = delete;
    void operator=(const DataStore&) = delete;

    template <typename T>
    void publish(const std::string& topic, const T& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Update current value
        data_map_[topic] = data;

        // Notify subscribers
        if (subscribers_.find(topic) != subscribers_.end()) {
            for (auto& cb : subscribers_[topic]) {
                cb(data);
            }
        }
    }

    template <typename T>
    void subscribe(const std::string& topic, std::function<void(const T&)> callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Wrap the typed callback into a generic std::any callback
        auto wrapper = [callback](const std::any& data) {
            try {
                callback(std::any_cast<T>(data));
            } catch (const std::bad_any_cast& e) {
                common::LogError("DataStore Type Mismatch on topic: {}", e.what());
            }
        };

        subscribers_[topic].push_back(wrapper);
    }

    // Get latest value directly (polling)
    template <typename T>
    bool get(const std::string& topic, T& out_data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_map_.find(topic) != data_map_.end()) {
            try {
                out_data = std::any_cast<T>(data_map_[topic]);
                return true;
            } catch (const std::bad_any_cast&) {
                return false;
            }
        }
        return false;
    }

    // Clear all data and subscribers (mainly for testing)
    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_map_.clear();
        subscribers_.clear();
    }

private:
    DataStore() = default;

    std::mutex mutex_;
    std::unordered_map<std::string, std::any> data_map_;
    std::unordered_map<std::string, std::vector<Callback>> subscribers_;
};

} // namespace fsw
