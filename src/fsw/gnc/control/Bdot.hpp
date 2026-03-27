#pragma once
#include "common/types.hpp"
#include "fsw/gnc/interfaces/IController.hpp"

namespace fsw {
namespace gnc {
namespace control {

/**
 * @brief B-Dot Magnetic Controller
 *
 * Implements the B-Dot control law: M = -K * B_dot
 * Use to detumble the spacecraft.
 */
class Bdot : public interfaces::IController {
   public:
    /**
     * @brief Constructor
     * @param gain Controller gain [Am^2 * s / T] typically.
     */
    explicit Bdot(double gain);

    /**
     * @brief Calculate control dipole moment
     * @param sensors Latest sensor measurements.
     * @param state Latest estimated state.
     * @param target Guidance target.
     * @param dt Time step [seconds]
     * @return Commanded dipole moment [Am^2] in Body frame
     */
    common::Vector3 update(const common::SensorData& sensors, const common::State& state_curr,
                           const common::GuidanceTarget& target, double dt) override;

    /**
     * @brief Reset the controller state (e.g. previous measurement)
     */
    void reset() override;

   private:
    double gain_;
    common::Vector3 b_prev_;
    bool first_run_;
};

}  // namespace control
}  // namespace gnc
}  // namespace fsw
