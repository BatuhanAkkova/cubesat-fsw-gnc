#include "fsw/gnc/control/Bdot.hpp"

namespace fsw {
namespace gnc {
namespace control {

Bdot::Bdot(double gain)
    : gain_(gain), b_prev_(common::Vector3::Zero()), first_run_(true) {}

common::Vector3 Bdot::update(const common::Vector3& b_body_T, double dt) {
    if (first_run_) {
        b_prev_ = b_body_T;
        first_run_ = false;
        return common::Vector3::Zero();
    }

    if (dt <= 1e-6) {
        // Avoid division by zero
        return common::Vector3::Zero();
    }

    common::Vector3 b_dot = (b_body_T - b_prev_) / dt;
    
    // Control Law: M = -K * B_dot
    common::Vector3 cmd = -gain_ * b_dot;

    // Update state
    b_prev_ = b_body_T;

    return cmd;
}

void Bdot::reset() {
    first_run_ = true;
    b_prev_ = common::Vector3::Zero();
}

} // namespace control
} // namespace gnc
} // namespace fsw
