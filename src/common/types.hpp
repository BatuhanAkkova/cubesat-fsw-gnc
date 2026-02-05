#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace common {

// Scalar constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

// Vector types
using Vector3 = Eigen::Vector3d;
using Vector4 = Eigen::Vector4d;
using VectorX = Eigen::VectorXd;
using Matrix3 = Eigen::Matrix3d;
using MatrixX = Eigen::MatrixXd;

// Quaternion type (Eigen stores complex part first: x, y, z, w internally, 
// but constructor is w, x, y, z)
using Quaternion = Eigen::Quaterniond;

} // namespace common
