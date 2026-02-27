#pragma once
#include "hal/interfaces/IGyro.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include <random>

namespace sim {
namespace models {

/**
 * @brief Simulated Gyroscope
 */
    /**
     * @brief Configuration for gyroscope model
     */
    struct SimGyroConfig {
        double noise_std;  // Standard deviation of noise [rad/s]
        common::Vector3 bias; // Constant bias [rad/s]
        
        SimGyroConfig() : noise_std(0.0), bias(common::Vector3::Zero()) {}
    };

class SimGyro : public hal::IGyro {
public:
    using Config = SimGyroConfig;

    explicit SimGyro(const dynamics::RigidBody& body, const Config& config = Config());

    common::Vector3 read() override;

    void injectFailure_Dead() { is_dead_ = true; }
    void setScalingFactor(double factor) { scaling_factor_ = factor; }

private:
    const dynamics::RigidBody& body_;
    Config config_;
    
    bool is_dead_ = false;
    double scaling_factor_ = 1.0;
    
    mutable std::mt19937 gen_;
    mutable std::normal_distribution<double> dist_;
};

} // namespace models
} // namespace sim
