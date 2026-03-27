#include <gtest/gtest.h>
#include "fsw/gnc/control/PID.hpp"

using namespace fsw::gnc::control;

TEST(PIDTest, ProportionalAction) {
    PID::Config cfg;
    cfg.kp = 2.0;
    cfg.ki = 0.0;
    cfg.kd = 0.0;
    cfg.limit = 10.0;
    cfg.anti_windup_limit = 10.0;
    
    PID pid(cfg);
    
    // error = 1.0, dt = 0.1
    // output = kp * error = 2.0
    double out = pid.calculate(1.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 2.0);
}

TEST(PIDTest, DerivativeAction) {
    PID::Config cfg;
    cfg.kp = 0.0;
    cfg.ki = 0.0;
    cfg.kd = 0.5;
    cfg.limit = 10.0;
    cfg.anti_windup_limit = 10.0;
    
    PID pid(cfg);
    
    // First call: init derivative
    pid.calculate(0.0, 0.1);
    
    // Second call: error jump 0.0 -> 1.0
    // dedt = (1.0 - 0.0) / 0.1 = 10.0
    // output = kd * 10.0 = 5.0
    double out = pid.calculate(1.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 5.0);
}

TEST(PIDTest, IntegralAction) {
    PID::Config cfg;
    cfg.kp = 0.0;
    cfg.ki = 1.5;
    cfg.kd = 0.0;
    cfg.anti_windup_limit = 10.0;
    
    PID pid(cfg);
    
    // error = 1.0, dt = 0.1
    // integral = 1.0 * 0.1 = 0.1
    // output = 1.5 * 0.1 = 0.15
    double out1 = pid.calculate(1.0, 0.1);
    EXPECT_DOUBLE_EQ(out1, 0.15);
    
    // Second step: integral += 1.0 * 0.1 = 0.2
    // output = 1.5 * 0.2 = 0.3
    double out2 = pid.calculate(1.0, 0.1);
    EXPECT_DOUBLE_EQ(out2, 0.3);
}

TEST(PIDTest, SaturationAndAntiWindup) {
    PID::Config cfg;
    cfg.kp = 10.0;
    cfg.ki = 5.0;
    cfg.kd = 0.0;
    cfg.limit = 1.0;
    cfg.anti_windup_limit = 0.5;
    
    PID pid(cfg);
    
    // Large error leads to saturation
    // out = 10 * 1.0 + 5 * 0.1 = 10.5 -> capped at 1.0
    double out = pid.calculate(1.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 1.0);
}
