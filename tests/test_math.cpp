#include <gtest/gtest.h>

#include "common/time.hpp"
#include "common/types.hpp"

using namespace common;

TEST(MathTest, VectorAliases) {
    Vector3 v(1.0, 2.0, 3.0);
    EXPECT_DOUBLE_EQ(v.x(), 1.0);
    EXPECT_DOUBLE_EQ(v.norm(), std::sqrt(14.0));
}

TEST(MathTest, QuaternionAliases) {
    Quaternion q = Quaternion::Identity();
    EXPECT_DOUBLE_EQ(q.w(), 1.0);
    EXPECT_DOUBLE_EQ(q.x(), 0.0);
}

TEST(TimeTest, SpacecraftTime) {
    SpacecraftTime t1(10.0);
    SpacecraftTime t2(15.0);

    EXPECT_TRUE(t2 > t1);

    auto t3 = t1 + 5.0;
    EXPECT_DOUBLE_EQ(t3.as_seconds(), 15.0);
}
