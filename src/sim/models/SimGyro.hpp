#pragma once
#include "hal/interfaces/IGyro.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include <random>

namespace sim {
namespace models {

/**
 * @brief Simulated Gyroscope
 */
class SimGyro : public hal::IGyro {
public:
    /**
     * @brief Configuration for gyroscope model
     */
    struct Config {
        double noise_std = 0.0;  // Standard deviation of noise [rad/s]
        common::Vector3 bias = common::Vector3::Zero(); // Constant bias [rad/s]
    };

    /**
     * @brief Constructor
     * @param body Reference to the simulated rigid body dynamics
     * @param config Gyro configuration
     */
    explicit SimGyro(const dynamics::RigidBody& body, const Config& config = Config());

    common::Vector3 read() override;

private:
    const dynamics::RigidBody& body_;
    Config config_;
    
    mutable std::mt19937 gen_;
    mutable std::normal_distribution<double> dist_;
};

} // namespace models
} // namespace sim
