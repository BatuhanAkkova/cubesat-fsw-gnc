#pragma once
#include "common/types.hpp"
#include "common/SensorData.hpp"

namespace fsw {
namespace gnc {
namespace interfaces {

/**
 * @brief Generic interface for GNC state estimators (EKF, UKF, etc.).
 */
class IEstimator {
public:
    virtual ~IEstimator() = default;

    /**
     * @brief Reset the internal state of the estimator.
     */
    virtual void reset() = 0;

    /**
     * @brief Update the estimator with new sensor data.
     */
    virtual void update(const common::SensorData& sensors, double dt) = 0;

    /**
     * @brief Get the latest attitude estimate.
     */
    virtual common::Quaternion getAttitude() const = 0;

    /**
     * @brief Get the latest angular velocity estimate.
     */
    virtual common::Vector3 getAngularVelocity() const = 0;

    /**
     * @brief Check if the estimator has converged or is valid.
     */
    virtual bool isValid() const = 0;
};

} // namespace interfaces
} // namespace gnc
} // namespace fsw
