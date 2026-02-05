#include <gtest/gtest.h>
#include "fsw/gnc/control/LQRController.hpp"
#include "fsw/gnc/control/LQGController.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"

using namespace fsw::gnc::control;
using namespace fsw::gnc::ekf;

/**
 * @brief Test LQR controller standalone torque calculation
 */
TEST(LQRControllerTest, BasicTorqueCalculation) {
    LQRController::Config cfg;
    cfg.K = common::MatrixX::Zero(3, 6);
    // Set diagonal gains
    for (int i = 0; i < 3; ++i) {
        cfg.K(i, i) = 1.0;     // Kp-like
        cfg.K(i, i + 3) = 0.5; // Kd-like
    }

    LQRController lqr(cfg);

    // Test case 1: Positive error in attitude, positive error in rate
    common::VectorX x(6);
    x << 1.0, 0.0, 0.0,  // delta_theta
         0.2, 0.0, 0.0;  // delta_omega

    common::Vector3 torque = lqr.computeTorque(x);

    // u = -K * x
    // u[0] = -(1.0 * 1.0 + 0.5 * 0.2) = -1.1
    EXPECT_NEAR(torque.x(), -1.1, 1e-6);
    EXPECT_NEAR(torque.y(), 0.0, 1e-6);
    EXPECT_NEAR(torque.z(), 0.0, 1e-6);
}

/**
 * @brief Test updating LQR gains at runtime
 */
TEST(LQRControllerTest, SetGains) {
    LQRController lqr(LQRController::Config::Default());
    
    common::MatrixX new_K = common::MatrixX::Ones(3, 6);
    lqr.setGains(new_K);
    
    common::VectorX x = common::VectorX::Ones(6);
    common::Vector3 torque = lqr.computeTorque(x);
    
    // u = -K * x = - [1,1,1,1,1,1] * [1,1,1,1,1,1]^T = -6 for each axis
    EXPECT_NEAR(torque.x(), -6.0, 1e-6);
    EXPECT_NEAR(torque.y(), -6.0, 1e-6);
    EXPECT_NEAR(torque.z(), -6.0, 1e-6);
}

/**
 * @brief Test LQG controller integration with MEKF
 */
TEST(LQGControllerTest, IntegrationWithMEKF) {
    auto mekf = std::make_shared<MEKF>();
    
    // Initialize MEKF with identity attitude and zero bias
    common::Quaternion q0 = common::Quaternion::Identity();
    common::Vector3 beta0 = common::Vector3::Zero();
    common::MatrixX P0 = common::MatrixX::Identity(6, 6) * 0.1;
    mekf->initialize(q0, beta0, P0);

    LQGController::Config cfg;
    cfg.lqr_cfg = LQRController::Config::Default(1.0, 0.5);
    LQGController lqg(mekf, cfg);

    // Case: Target is rotated 5 degrees about X relative to current estimate (Identity)
    // Note: q_target is Inertial -> Body
    common::Quaternion q_target(common::AngleAxis(5.0 * common::DEG2RAD, common::Vector3::UnitX()));
    common::Vector3 omega_target = common::Vector3::Zero();
    common::Vector3 omega_meas = common::Vector3::Zero();

    common::Vector3 torque = lqg.computeTorque(q_target, omega_target, omega_meas);

    // q_est is Identity
    // q_err = q_est * q_target_inv = Identity * q_target_inv = q_target_inv
    // q_target_inv is a -5 degree rotation about X
    // delta_theta = 2 * vec(q_err) = 2 * [sin(-2.5 deg), 0, 0] approx [-0.087, 0, 0]
    // u = -K * x = - [1.0 * delta_theta[0]] = - (-0.087) = 0.087
    
    EXPECT_GT(torque.x(), 0.0);
    EXPECT_NEAR(torque.y(), 0.0, 1e-6);
    EXPECT_NEAR(torque.z(), 0.0, 1e-6);
}
