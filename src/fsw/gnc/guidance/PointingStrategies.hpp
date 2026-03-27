#pragma once

#include <cmath>

#include "common/types.hpp"

namespace fsw {
namespace gnc {
namespace guidance {

/**
 * @brief Utilities to calculate target quaternions from various constraints.
 */
class PointingStrategies {
   public:
    /**
     * @brief Create a target quaternion that aligns a body axis with an inertial vector.
     *
     * @param body_axis Vector in Body frame to align (e.g. [1, 0, 0] for X-face).
     * @param inertial_target Vector in Inertial frame to point at (e.g. Nadir, Sun).
     * @return common::Quaternion Target attitude (Body to Inertial).
     */
    static common::Quaternion alignAxis(const common::Vector3& body_axis, const common::Vector3& inertial_target) {
        common::Vector3 b = body_axis.normalized();
        common::Vector3 i = inertial_target.normalized();

        // Find rotation from b to i
        // q = [cos(theta/2), sin(theta/2) * axis]
        // axis = b x i / |b x i|
        // cos(theta) = b . i

        common::Vector3 cross = b.cross(i);
        double dot = b.dot(i);

        // Handle parallel/anti-parallel cases
        if (cross.norm() < 1e-9) {
            if (dot > 0) return common::Quaternion::Identity();
            // Anti-parallel: rotate 180 deg about any orthogonal axis
            common::Vector3 ortho = (std::abs(b.x()) < 0.9) ? common::Vector3(1, 0, 0) : common::Vector3(0, 1, 0);
            common::Vector3 axis = b.cross(ortho).normalized();
            return common::Quaternion(common::AngleAxis(common::PI, axis));
        }

        // Standard case:
        // Use Eigen's FromTwoVectors or manual construction
        return common::Quaternion::FromTwoVectors(b, i);
    }

    /**
     * @brief Constrained alignment (e.g. Point X at Sun, and keep Y near Nadir).
     * This is more complex, usually requires a triad-like approach.
     */
    static common::Quaternion nadirPointing(const common::Vector3& sc_pos_eci, const common::Vector3& sc_vel_eci) {
        // -Z face towards Earth (-pos)
        // Y face towards Orbit Normal (pos x vel)
        // X face towards velocityish

        common::Vector3 z_body = -sc_pos_eci.normalized();
        common::Vector3 y_body = (sc_pos_eci.cross(sc_vel_eci)).normalized();
        common::Vector3 x_body = y_body.cross(z_body).normalized();

        // Construct DCM [X|Y|Z] from Body to Inertial
        common::Matrix3 dcm;
        dcm.col(0) = x_body;
        dcm.col(1) = y_body;
        dcm.col(2) = z_body;

        return common::Quaternion(dcm);
    }
};

}  // namespace guidance
}  // namespace gnc
}  // namespace fsw
