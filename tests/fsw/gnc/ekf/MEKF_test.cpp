#include <gtest/gtest.h>
#include "fsw/gnc/ekf/MEKF.hpp"
#include <iostream>

using namespace fsw::gnc::ekf;
using namespace common;

class MEKFTest : public ::testing::Test {
protected:
    MEKF mekf;
    
    void SetUp() override {
        // Default init
    }
};

TEST_F(MEKFTest, Initialization) {
    Quaternion q0 = Quaternion(Eigen::AngleAxisd(0.5, Vector3::UnitZ()));
    Vector3 b0(0.01, -0.01, 0.005);
    MatrixX P0 = MatrixX::Identity(6, 6);
    
    mekf.initialize(q0, b0, P0);
    
    EXPECT_TRUE(mekf.getAttitude().isApprox(q0));
    EXPECT_TRUE(mekf.getBias().isApprox(b0));
}

TEST_F(MEKFTest, PredictionStatic) {
    // Static spacecraft, perfect gyro (reading 0)
    Quaternion q0 = Quaternion::Identity();
    Vector3 b0 = Vector3::Zero();
    MatrixX P0 = MatrixX::Identity(6, 6) * 0.1;
    
    mekf.initialize(q0, b0, P0);
    
    // Perfect gyro reading = 0
    // Bias estimate = 0
    // Expected: No change
    mekf.predict(Vector3::Zero(), 0.1, MatrixX::Identity(6, 6) * 1e-4);
    
    EXPECT_TRUE(mekf.getAttitude().isApprox(q0));
    EXPECT_TRUE(mekf.getBias().isApprox(b0));
}

TEST_F(MEKFTest, UpdateReduction) {
    // Initialize with error
    Quaternion true_q = Quaternion::Identity();
    
    // Initial estimate has error about X
    Quaternion initial_est = Quaternion(Eigen::AngleAxisd(0.1, Vector3::UnitX())); 
    
    mekf.initialize(initial_est, Vector3::Zero(), MatrixX::Identity(6, 6) * 1.0);
    
    // Low measurement noise
    Matrix3 R = Matrix3::Identity() * 1e-4;
    
    // Update with "measurement" = true_q
    mekf.update_quat(true_q, R);
    
    // Error should decrease
    // initial error is 0.1 rad
    // check angle between est and true
    Quaternion q_diff = mekf.getAttitude() * true_q.inverse();
    double angular_error = Eigen::AngleAxisd(q_diff).angle();
    
    EXPECT_LT(angular_error, 0.1);
    // With P=1, R=1e-4, K ~ 1, so it should jump almost to truth
    EXPECT_NEAR(angular_error, 0.0, 1e-3);
}

TEST_F(MEKFTest, BiasEstimation) {
    // True state: Rotating about Z at 1 rad/s
    // True Bias: [0.1, 0, 0]
    Vector3 true_bias(0.1, 0.0, 0.0);
    Vector3 true_rate(0.0, 0.0, 1.0); // Inertial rate
    
    // Initial Estimate
    Quaternion q_est = Quaternion::Identity();
    Vector3 b_est = Vector3::Zero(); // Wrong bias
    MatrixX P0 = MatrixX::Identity(6, 6) * 0.1;
    P0.block<3,3>(3,3) *= 1.0; // High uncertainty in bias
    
    mekf.initialize(q_est, b_est, P0);
    
    double dt = 0.01;
    MatrixX Q = MatrixX::Identity(6, 6) * 1e-5;
    Matrix3 R = Matrix3::Identity() * 1e-4;
    
    Quaternion true_q = Quaternion::Identity();
    
    // Run loop
    for (int i = 0; i < 500; ++i) {
        // Simulate Reality
        // Static Spacecraft, Gyro reads Bias
        // True Omega = 0
        // Gyro Meas = True Omega + Bias + Noise = Bias
        Vector3 gyro_meas = true_bias; 
        
        // Predict
        mekf.predict(gyro_meas, dt, Q);
        
        // Update (Measurement is constant Identity)
        mekf.update_quat(Quaternion::Identity(), R);
    }
    
    // Filter should learn that the gyro reading is actually bias
    // omega_est = gyro_meas - beta_est => beta_est should -> gyro_meas
    
    Vector3 final_bias = mekf.getBias();
    EXPECT_NEAR(final_bias.x(), 0.1, 0.02);
    EXPECT_NEAR(final_bias.y(), 0.0, 0.01);
    EXPECT_NEAR(final_bias.z(), 0.0, 0.01);
}

TEST_F(MEKFTest, LostInSpace) {
    // Large initial error and high uncertainty
    Quaternion true_q = Quaternion::Identity();
    Quaternion initial_est = Quaternion(AngleAxis(PI/2, Vector3::UnitY())); // 90 deg error
    
    MatrixX P0 = MatrixX::Identity(6, 6) * 2.0; // High uncertainty
    mekf.initialize(initial_est, Vector3::Zero(), P0);
    
    Matrix3 R = Matrix3::Identity() * 0.01;
    
    // Perform several updates with truth
    for (int i = 0; i < 20; ++i) {
        mekf.update_quat(true_q, R);
        
        // Mock predict with zero motion
        mekf.predict(Vector3::Zero(), 0.1, MatrixX::Identity(6, 6) * 1e-6);
    }
    
    double error = AngleAxis(mekf.getAttitude() * true_q.inverse()).angle();
    EXPECT_LT(error, 0.05); // Should converge
    
    // Covariance should decrease
    EXPECT_LT(mekf.getCovariance().trace(), P0.trace());
}

TEST_F(MEKFTest, BiasWalk) {
    // Verify filter can track a slowly changing bias
    mekf.initialize(Quaternion::Identity(), Vector3::Zero(), MatrixX::Identity(6, 6) * 0.1);
    
    double dt = 0.1;
    MatrixX Q = MatrixX::Identity(6, 6) * 1e-4; // High process noise for bias
    Matrix3 R = Matrix3::Identity() * 0.01;
    
    Vector3 walking_bias(0.1, 0.0, 0.0);
    
    for (int i = 0; i < 200; ++i) {
        walking_bias.x() += 0.0001; // Slow drift
        
        mekf.predict(walking_bias, dt, Q);
        mekf.update_quat(Quaternion::Identity(), R);
    }
    
    EXPECT_NEAR(mekf.getBias().x(), walking_bias.x(), 0.01);
}
