#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <algorithm>

namespace fsw {
namespace telemetry {

/**
 * @brief Utility to read parameters from a command payload.
 * Handles Big-Endian to Host conversion.
 */
class CommandParameterReader {
public:
    CommandParameterReader(const uint8_t* data, size_t size) 
        : data_(data), size_(size), offset_(0) {}

    /**
     * @brief Read a value of type T from the payload.
     * @tparam T The type to read.
     * @return The read value.
     */
    template<typename T>
    T read() {
        if (offset_ + sizeof(T) > size_) {
            return T{}; 
        }
        
        T val;
        uint8_t temp[sizeof(T)];
        std::memcpy(temp, data_ + offset_, sizeof(T));
        
        // CCSDS is Big-Endian. If host is Little-Endian, we reverse.
        // For simplicity, we assume Little-Endian host and reverse.
#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        // No change needed
#else
        std::reverse(temp, temp + sizeof(T));
#endif
        std::memcpy(&val, temp, sizeof(T));
        
        offset_ += sizeof(T);
        return val;
    }

    /**
     * @brief Read a boolean value (1 byte).
     */
    bool readBool() {
        if (offset_ + 1 > size_) return false;
        return data_[offset_++] != 0;
    }

    /**
     * @brief Get remaining bytes in payload.
     */
    size_t getRemainingSize() const { return size_ - offset_; }

private:
    const uint8_t* data_;
    size_t size_;
    size_t offset_;
};

} // namespace telemetry
} // namespace fsw
