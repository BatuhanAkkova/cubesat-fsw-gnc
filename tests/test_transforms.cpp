#include <gtest/gtest.h>
#include "fsw/gnc/guidance/PointingStrategies.hpp"
#include <cmath>

using namespace fsw::gnc::guidance;
using namespace common;

/**
 * @brief Test PointingStrategies::alignAxis
 */
TEST(PointingTest, AlignAxisBasic) {
    // Case 1: Standard alignment
    Vector3 body_x(1, 0, 0);
    Vector3 target_eci(0, 1, 0);
    
    Quaternion q = PointingStrategies::alignAxis(body_x, target_eci);
    
    // Verify: q * body_x should be target_eci
    Vector3 aligned = q * body_x;
    EXPECT_NEAR(aligned.x(), 0.0, 1e-6);
    EXPECT_NEAR(aligned.y(), 1.0, 1e-6);
    EXPECT_NEAR(aligned.z(), 0.0, 1e-6);
}

TEST(PointingTest, AlignAxisParallel) {
    // Case 2: Already aligned
    Vector3 body_z(0, 0, 1);
    Vector3 target_z(0, 0, 1);
    
    Quaternion q = PointingStrategies::alignAxis(body_z, target_z);
    
    EXPECT_TRUE(q.isApprox(Quaternion::Identity(), 1e-9));
}

TEST(PointingTest, AlignAxisAntiParallel) {
    // Case 3: 180 degree rotation required
    Vector3 body_y(0, 1, 0);
    Vector3 target_neg_y(0, -1, 0);
    
    Quaternion q = PointingStrategies::alignAxis(body_y, target_neg_y);
    
    Vector3 aligned = q * body_y;
    EXPECT_NEAR(aligned.x(), 0.0, 1e-6);
    EXPECT_NEAR(aligned.y(), -1.0, 1e-6);
    EXPECT_NEAR(aligned.z(), 0.0, 1e-6);
    
    // Check it's a 180 deg rotation
    EXPECT_NEAR(std::abs(AngleAxis(q).angle()), PI, 1e-6);
}

/**
 * @brief Test PointingStrategies::nadirPointing
 */
TEST(PointingTest, NadirPointing) {
    // Simulation: Sc at [R, 0, 0], Vel at [0, V, 0]
    // Nadir (-pos) is [-1, 0, 0]
    // Orbit Normal (pos x vel) is [0, 0, 1]
    
    Vector3 pos(7000e3, 0, 0);
    Vector3 vel(0, 7500, 0);
    
    Quaternion q = PointingStrategies::nadirPointing(pos, vel);
    
    // Body -Z should be Nadir [-1, 0, 0]
    Vector3 body_z(0, 0, 1);
    Vector3 aligned_z = q * body_z;
    EXPECT_NEAR(aligned_z.x(), -1.0, 1e-6);
    EXPECT_NEAR(aligned_z.y(), 0.0, 1e-6);
    EXPECT_NEAR(aligned_z.z(), 0.0, 1e-6);
    
    // Body Y should be Orbit Normal [0, 0, 1]
    Vector3 body_y(0, 1, 0);
    Vector3 aligned_y = q * body_y;
    EXPECT_NEAR(aligned_y.x(), 0.0, 1e-6);
    EXPECT_NEAR(aligned_y.y(), 0.0, 1e-6);
    EXPECT_NEAR(aligned_y.z(), 1.0, 1e-6);
}
