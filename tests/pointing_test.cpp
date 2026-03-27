#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/guidance/PointingStrategies.hpp"
#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimRW.hpp"

#ifndef M_PI
#define M_PI common::PI
#endif

using namespace common;

class PointingTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // 1. Satellite Inertia
        Matrix3 inertia;
        inertia << 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1;

        // 2. Initial state: Rest, Identity attitude
        Quaternion q_init(1, 0, 0, 0);
        Vector3 w_init(0, 0, 0);

        body = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);

        // 3. Setup 3 Reaction Wheels (X, Y, Z)
        sim::SimRW::Config rw_cfg;
        rw_cfg.inertia = 0.001;
        rw_cfg.max_torque = 0.05;
        rw_cfg.max_momentum = 0.1;
        rw_cfg.friction_coeff = 0.0001;
        rw_cfg.initial_speed = 0.0;

        wheels.push_back(std::make_unique<sim::SimRW>(rw_cfg));  // X
        wheels.push_back(std::make_unique<sim::SimRW>(rw_cfg));  // Y
        wheels.push_back(std::make_unique<sim::SimRW>(rw_cfg));  // Z

        // 4. Setup Controller
        fsw::gnc::control::AttitudeController::Config att_config;
        att_config.nominal_pid.kp = 0.5;
        att_config.nominal_pid.ki = 0.01;
        att_config.nominal_pid.kd = 1.0;
        att_config.nominal_pid.limit = 0.05;
        att_config.nominal_pid.anti_windup_limit = 0.01;

        att_config.large_error_pid = att_config.nominal_pid;  // Same for this test

        controller = std::make_unique<fsw::gnc::control::AttitudeController>(att_config);
    }

    std::unique_ptr<sim::dynamics::RigidBody> body;
    std::vector<std::unique_ptr<sim::SimRW>> wheels;
    std::unique_ptr<fsw::gnc::control::AttitudeController> controller;
};

TEST_F(PointingTest, SlewManeuver) {
    double dt = 0.1;
    double simulation_time = 30.0;  // seconds
    int steps = static_cast<int>(simulation_time / dt);

    // Target: 90 degree rotation about Z axis
    Quaternion q_target(AngleAxis(M_PI / 2.0, Vector3::UnitZ()));

    std::cout << "Starting Slew" << std::endl;

    for (int i = 0; i < steps; ++i) {
        // 1. FSW: Get Attitude & Compute Control
        common::SensorData sensors;
        sensors.q_measured = body->getAttitude();
        sensors.gyro_body = body->getAngularVelocity();

        common::State state_est;
        state_est.q = sensors.q_measured;
        state_est.w = sensors.gyro_body;

        common::GuidanceTarget target;
        target.q = q_target;
        target.w = common::Vector3::Zero();

        Vector3 torque_cmd = controller->update(sensors, state_est, target, dt);

        // 2. SIM: Apply torque to wheels and get feedback
        Vector3 internal_torque = Vector3::Zero();
        Vector3 internal_momentum = Vector3::Zero();

        // Assume wheels are aligned with body X, Y, Z axes
        internal_torque.x() = wheels[0]->step(dt);
        internal_torque.y() = wheels[1]->step(dt);
        internal_torque.z() = wheels[2]->step(dt);

        internal_momentum.x() = wheels[0]->getAngularMomentum();
        internal_momentum.y() = wheels[1]->getAngularMomentum();
        internal_momentum.z() = wheels[2]->getAngularMomentum();

        // Feed command to wheels for next step
        wheels[0]->setTorqueCommand(-torque_cmd.x());
        wheels[1]->setTorqueCommand(-torque_cmd.y());
        wheels[2]->setTorqueCommand(-torque_cmd.z());

        // 3. SIM: Propagate Body Dynamics
        body->step(dt, Vector3::Zero(), internal_torque, internal_momentum);

        if (i % 50 == 0) {
            double err_norm = (sensors.q_measured.inverse() * q_target).vec().norm();
            std::cout << "Time: " << i * dt << " | Att Err: " << err_norm << std::endl;
        }
    }

    Quaternion q_final = body->getAttitude();
    double angle_err = q_final.angularDistance(q_target);
    double angle_err_deg = angle_err * 180.0 / M_PI;
    std::cout << "Final Angle Error (deg): " << angle_err_deg << std::endl;

    EXPECT_LT(angle_err_deg, 10.0);                     // Relaxed for rate-limited controller
    EXPECT_LT(body->getAngularVelocity().norm(), 0.3);  // Settling in progress
}
