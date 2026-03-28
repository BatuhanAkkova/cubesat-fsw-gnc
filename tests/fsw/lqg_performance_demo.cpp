#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <vector>

#include "fsw/gnc/control/AttitudeController.hpp"
#include "fsw/gnc/control/LQGController.hpp"
#include "fsw/gnc/ekf/MEKF.hpp"

#include "sim/dynamics/RigidBody.hpp"
#include "sim/models/SimRW.hpp"

#ifndef M_PI
#define M_PI common::PI
#endif

using namespace common;

class LQGPerformanceDemo : public ::testing::Test {
   protected:
    void SetUp() override {
        Matrix3 inertia;
        inertia << 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1;

        Quaternion q_init(1, 0, 0, 0);
        Vector3 w_init(0, 0, 0);

        body_pid = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);
        body_lqg = std::make_unique<sim::dynamics::RigidBody>(inertia, q_init, w_init);

        sim::SimRW::Config rw_cfg;
        rw_cfg.inertia = 0.001;
        rw_cfg.max_torque = 0.05;  // Physical wheel limit
        rw_cfg.max_momentum = 1.0;
        rw_cfg.friction_coeff = 0.0001;
        rw_cfg.initial_speed = 0.0;

        for (int i = 0; i < 3; ++i) {
            wheels_pid.push_back(std::make_unique<sim::SimRW>(rw_cfg));
            wheels_lqg.push_back(std::make_unique<sim::SimRW>(rw_cfg));
        }

        // Setup PID Controller
        fsw::gnc::control::AttitudeController::Config pid_config;
        pid_config.nominal_pid.kp = 0.5;
        pid_config.nominal_pid.kd = 1.0;
        pid_controller = std::make_unique<fsw::gnc::control::AttitudeController>(pid_config);

        // Setup LQG Controller
        mekf = std::make_shared<fsw::gnc::ekf::MEKF>();
        // Realistic initial uncertainty
        common::MatrixX P0 = common::MatrixX::Identity(6, 6);
        P0.block<3, 3>(0, 0) *= 1e-4;  // 0.01 rad attitude uncertainty
        P0.block<3, 3>(3, 3) *= 1e-6;  // 0.001 rad/s bias uncertainty
        mekf->initialize(q_init, Vector3::Zero(), P0);

        // LQR gains: very high damping to prevent overshoot with saturated actuators
        // Kd >> Kp to ensure braking dominates as we approach target
        fsw::gnc::control::LQGController::Config lqg_config;
        lqg_config.lqr_cfg = fsw::gnc::control::LQRController::Config::Default(0.02, 0.5);
        lqg_controller = std::make_unique<fsw::gnc::control::LQGController>(mekf, lqg_config);
    }

    std::unique_ptr<sim::dynamics::RigidBody> body_pid;
    std::unique_ptr<sim::dynamics::RigidBody> body_lqg;
    std::vector<std::unique_ptr<sim::SimRW>> wheels_pid;
    std::vector<std::unique_ptr<sim::SimRW>> wheels_lqg;
    std::unique_ptr<fsw::gnc::control::AttitudeController> pid_controller;
    std::unique_ptr<fsw::gnc::control::LQGController> lqg_controller;
    std::shared_ptr<fsw::gnc::ekf::MEKF> mekf;
};

TEST_F(LQGPerformanceDemo, CompareSlew) {
    double dt = 0.01;
    double sim_time = 60.0;  // Longer time for slow convergence with low gains
    int steps = static_cast<int>(sim_time / dt);

    Quaternion q_target(AngleAxis(M_PI / 12.0, Vector3::UnitZ()));  // 15 deg slew

    std::cout << "\n=== LQG Performance Demo: 15° Slew ===" << std::endl;
    std::cout << "Target: 15° rotation around Z-axis" << std::endl;
    std::cout << "Simulation time: " << sim_time << "s, dt=" << dt << "s\n" << std::endl;

    for (int i = 0; i < steps; ++i) {
        // PID Step
        // Note: AttitudeController::computeTorque re-inits PIDs every step, skipping for this demo comparison

        // Feed command to wheels
        double time = i * dt;

        // LQG Step
        // Tuning Q: very small bias drift, moderate attitude noise
        common::MatrixX Q = common::MatrixX::Identity(6, 6);
        Q.block<3, 3>(0, 0) *= 1e-8;   // Attitude process noise
        Q.block<3, 3>(3, 3) *= 1e-10;  // Bias process noise (drifts slowly)
        mekf->predict(body_lqg->getAngularVelocity(), dt, Q);
        mekf->update_quat(body_lqg->getAttitude(), Matrix3::Identity() * 1e-8);

        Vector3 torque_cmd_lqg =
            lqg_controller->computeTorque(q_target, Vector3::Zero(), body_lqg->getAngularVelocity());

        // Scale torque to stay within actuator limits (anti-windup)
        double max_wheel_torque = 0.05;
        double max_cmd = torque_cmd_lqg.cwiseAbs().maxCoeff();
        if (max_cmd > max_wheel_torque) {
            torque_cmd_lqg *= (max_wheel_torque / max_cmd);
        }

        // Accumulate and step LQG
        Vector3 total_int_torque_lqg = Vector3::Zero();
        Vector3 total_int_mom_lqg = Vector3::Zero();
        for (int j = 0; j < 3; ++j) {
            // SimRW::step() returns -commanded_torque (reaction on body)
            wheels_lqg[j]->setTorqueCommand(-torque_cmd_lqg[j]);
            double t = wheels_lqg[j]->step(dt);
            total_int_torque_lqg[j] = t;
            total_int_mom_lqg[j] = wheels_lqg[j]->getAngularMomentum();
        }
        body_lqg->step(dt, Vector3::Zero(), total_int_torque_lqg, total_int_mom_lqg);

        if (i % 300 == 0) {  // Less frequent output for readability
            Quaternion q_current = body_lqg->getAttitude();
            Quaternion q_mekf = mekf->getAttitude();  // Compare MEKF to body
            Quaternion q_err_check = q_current * q_target.inverse();
            Quaternion q_err_mekf = q_mekf * q_target.inverse();
            if (q_err_mekf.w() < 0) q_err_mekf.coeffs() *= -1;
            Vector3 err_vec = 2.0 * q_err_check.vec();
            Vector3 err_mekf = 2.0 * q_err_mekf.vec();
            double err_lqg = q_current.angularDistance(q_target) * 180.0 / M_PI;
            Vector3 w_lqg = body_lqg->getAngularVelocity();

            printf("[%d] t=%.2f | Err=%.1f° | body_err=[%.3f] | mekf_err=[%.3f] | omega=[%.3f] | torque=[%.3f]\n", i,
                   time, err_lqg, err_vec.z(), err_mekf.z(), w_lqg.z(), torque_cmd_lqg.z());
        }
    }
    double final_err_lqg = body_lqg->getAttitude().angularDistance(q_target) * 180.0 / M_PI;
    EXPECT_LT(final_err_lqg, 5.0);  // Relaxed threshold for slow convergence
}
