#pragma once
#include "common/types.hpp"

namespace sim {
namespace dynamics {

class RigidBody {
   public:
    /**
     * @brief Constructor
     * @param inertia Inertia tensor [kg*m^2]
     * @param init_att Initial attitude quaternion (from Body to Inertial)
     * @param init_omega Initial angular velocity [rad/s] in Body frame
     */
    RigidBody(const common::Matrix3& inertia, const common::Quaternion& init_att, const common::Vector3& init_omega);

    /**
     * @brief Propagate dynamics by one time step
     * @param dt Time step [s]
     * @param external_torque External torque vector [Nm] in Body frame
     * @param internal_torque Torque from internal components (e.g. RW) [Nm] in Body frame
     * @param internal_momentum Momentum of internal components [Nms] in Body frame
     */
    void step(double dt, const common::Vector3& external_torque,
              const common::Vector3& internal_torque = common::Vector3::Zero(),
              const common::Vector3& internal_momentum = common::Vector3::Zero());

    common::Quaternion getAttitude() const;
    common::Vector3 getAngularVelocity() const;

   private:
    common::Matrix3 inertia_;
    common::Matrix3 inertia_inv_;

    // State: [qx, qy, qz, qw, wx, wy, wz] (7 elements)
    // Note: Eigen stores Quaternion as x, y, z, w
    common::VectorX state_;

    common::VectorX dynamics(double t, const common::VectorX& y, const common::Vector3& external_torque,
                             const common::Vector3& internal_torque, const common::Vector3& internal_momentum);
};

}  // namespace dynamics
}  // namespace sim
