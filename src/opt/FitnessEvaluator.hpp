#pragma once

#include "sim/Simulation.hpp"
#include "fsw/FlightSoftware.hpp"
#include <vector>
#include <cmath>

namespace opt {

/**
 * @brief Evaluates the fitness of a set of controller gains.
 */
class FitnessEvaluator {
public:
    struct Results {
        double cost;
        double settling_time;
        double max_overshoot;
        double energy_used;
    };

    /**
     * @brief Evaluate gains (kp, ki, kd) for nominal attitude control.
     * @param gains Vector [kp, ki, kd]
     * @param duration Simulation duration [s]
     * @return Results
     */
    Results evaluate(const std::vector<double>& gains, double duration = 60.0) {
        // 1. Setup Simulation
        sim::Simulation::Config sim_cfg;
        // Start with a 30 degree error on X-axis
        sim_cfg.init_q = common::Quaternion(common::AngleAxis(30.0 * common::DEG2RAD, common::Vector3::UnitX()));
        sim_cfg.init_w = common::Vector3(0.01, -0.01, 0.005); // Small initial tumble
        
        sim::Simulation sim(sim_cfg);

        // 2. Setup FSW
        fsw::FlightSoftware::Config fsw_cfg;
        // Start in Nominal mode to test pointing immediately
        fsw_cfg.mode_cfg.safe_to_nominal_rate_threshold = 1.0; 
        
        // Setup JSON config for factory to use
        fsw_cfg.full_config = {
            {"fsw", {
                {"controllers", {
                    {"attitude", {
                        {"type", "PID"},
                        {"nominal_pid", {{"kp", gains[0]}, {"ki", gains[1]}, {"kd", gains[2]}}}
                    }}
                }}
            }}
        };
        
        fsw_cfg.sun_inertial = common::Vector3(1, 0, 0); // Point at X axis
        fsw::FlightSoftware fsw(fsw_cfg);

        // 3. Run Simulation
        double dt = 0.1;
        double total_error_integral = 0.0;
        double total_energy = 0.0;
        double max_error = 0.0;
        
        bool stable = true;

        for (double t = 0; t < duration; t += dt) {
            common::SensorData sensors = sim.getSensors();
            std::vector<std::vector<uint8_t>> empty_cmds;
            common::Vector3 torque = fsw.step(sensors, empty_cmds, dt);
            sim.step(dt, torque);

            // Calculate current pointing error (angle to Sun)
            common::Vector3 body_z = sim.getAttitude() * common::Vector3(0, 0, 1);
            common::Vector3 sun_inertial(1, 0, 0);
            double cos_theta = body_z.dot(sun_inertial);
            double error_rad = std::acos(std::clamp(cos_theta, -1.0, 1.0));
            
            total_error_integral += error_rad * dt;
            total_energy += torque.squaredNorm() * dt;
            
            if (t > 0.1) { // Ignore initial transient for overshoot
                 max_error = std::max(max_error, error_rad);
            }

            // Failure criteria: if error grows too large or NaN
            if (error_rad > 3.0 || std::isnan(error_rad)) {
                stable = false;
                break;
            }
        }

        Results res;
        if (!stable) {
            res.cost = 1e12; // Massively penalized
            return res;
        }

        // 4. Weighted Cost Function
        // heavily penalize error, moderately penalize energy
        res.cost = (100.0 * total_error_integral) + (10.0 * total_energy);
        res.energy_used = total_energy;
        // ... could calculate settling time here ...
        
        return res;
    }
};

} // namespace opt
