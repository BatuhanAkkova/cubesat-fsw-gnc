#pragma once

#include <Eigen/Dense>
#include <vector>

#include "common/types.hpp"

namespace common {

/**
 * @brief SIMD-optimized state history management using Structure-of-Arrays (SoA).
 *
 * Internally uses a ColMajor Eigen Matrix where each column represents a specific
 * state component (q_w, q_x, ..., vel_z) across all time steps.
 * This layout allows Eigen to use SIMD instructions for batch operations like
 * averaging, filtering, or coordinate transformations.
 */
class StateHistory {
   public:
    enum Component : int {
        QW = 0, QX, QY, QZ,
        WX, WY, WZ,
        POSX, POSY, POSZ,
        VELX, VELY, VELZ,
        NUM_COMPONENTS = 13
    };

    explicit StateHistory(size_t max_size) : max_size_(max_size), current_size_(0), head_(0) {
        // Pre-allocate matrix (Rows = Max History, Cols = 13 components)
        // ColMajor ensures that each component's history is contiguous in memory.
        data_ = Eigen::Matrix<double, Eigen::Dynamic, NUM_COMPONENTS>::Zero(max_size, NUM_COMPONENTS);
    }

    /**
     * @brief Add a new state to the history (circular buffer).
     */
    void addState(const State& state) {
        data_(head_, QW) = state.q.w();
        data_(head_, QX) = state.q.x();
        data_(head_, QY) = state.q.y();
        data_(head_, QZ) = state.q.z();
        data_(head_, WX) = state.w.x();
        data_(head_, WY) = state.w.y();
        data_(head_, WZ) = state.w.z();
        data_(head_, POSX) = state.pos.x();
        data_(head_, POSY) = state.pos.y();
        data_(head_, POSZ) = state.pos.z();
        data_(head_, VELX) = state.vel.x();
        data_(head_, VELY) = state.vel.y();
        data_(head_, VELZ) = state.vel.z();

        head_ = (head_ + 1) % max_size_;
        if (current_size_ < max_size_) current_size_++;
    }

    /**
     * @brief Get the history of a specific component as a SIMD-friendly Eigen vector.
     */
    Eigen::VectorXd getComponentHistory(Component comp) const {
        return data_.col(comp).head(current_size_);
    }

    /**
     * @brief Perform SIMD-optimized mean calculation for a component.
     */
    double mean(Component comp) const {
        if (current_size_ == 0) return 0.0;
        return data_.col(comp).head(current_size_).mean();
    }

    /**
     * @brief Perform SIMD-optimized standard deviation calculation.
     */
    double stddev(Component comp) const {
        if (current_size_ < 2) return 0.0;
        auto slice = data_.col(comp).head(current_size_);
        double m = slice.mean();
        return std::sqrt((slice.array() - m).square().sum() / (current_size_ - 1));
    }

    size_t size() const { return current_size_; }

   private:
    size_t max_size_;
    size_t current_size_;
    size_t head_;
    
    // Rows = Time index, Cols = Component index
    // ColMajor storage: Each column is contiguous = SoA layout!
    Eigen::Matrix<double, Eigen::Dynamic, NUM_COMPONENTS, Eigen::ColMajor> data_;
};

}  // namespace common
