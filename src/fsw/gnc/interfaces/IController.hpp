#pragma once
#include "common/SensorData.hpp"
#include "common/types.hpp"

namespace fsw {
namespace gnc {
namespace interfaces {

/**
 * @brief Generic interface for GNC controllers.
 */
class IController {
   public:
    virtual ~IController() = default;

    /**
     * @brief Reset the internal state of the controller.
     */
    virtual void reset() = 0;

    /**
     * @brief Update the controller.
     * @param sensors Latest sensor measurements.
     * @param state Latest estimated state.
     * @param target Guidance target.
     * @param dt Time step [s].
     * @return Commanded torque in body frame (Nm).
     */
    virtual common::Vector3 update(const common::SensorData& sensors, const common::State& state,
                                   const common::GuidanceTarget& target, double dt) = 0;

    /**
     * @brief Check if the controller is initialized.
     */
    virtual bool isInitialized() const {
        return true;
    }
};

}  // namespace interfaces
}  // namespace gnc
}  // namespace fsw
