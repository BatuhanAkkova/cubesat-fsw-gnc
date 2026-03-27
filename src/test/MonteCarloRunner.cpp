#include "test/MonteCarloRunner.hpp"
#include <iomanip>
#include <chrono>

namespace test {

MonteCarloRunner::MonteCarloRunner(const Config& config) 
    : config_(config), 
      gen_(std::chrono::system_clock::now().time_since_epoch().count()) {}

void MonteCarloRunner::run() {
    int successes = 0;
    std::vector<MonteCarloResult> results;

    std::cout << "\n=== STARTING MONTE CARLO ANALYSIS (" << config_.num_runs << " runs) ===" << std::endl;

    for (int i = 0; i < config_.num_runs; ++i) {
        MonteCarloResult res = runSingleSimulation(i);
        results.push_back(res);
        if (res.success) successes++;

        if ((i + 1) % 10 == 0) {
            std::cout << "Progress: " << (i + 1) << "/" << config_.num_runs 
                      << " | Current Success Rate: " << (double)successes / (i + 1) * 100.0 << "%" << std::endl;
        }
    }

    std::cout << "\n=== MONTE CARLO RESULTS ===" << std::endl;
    std::cout << "Overall Success Rate: " << (double)successes / config_.num_runs * 100.0 << "%" << std::endl;
    
    // Summary of failures (could be more detailed)
}

MonteCarloResult MonteCarloRunner::runSingleSimulation(int seed) {
    std::mt19937 run_gen(seed);
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::normal_distribution<double> norm(1.0, config_.inertia_variation / 2.0);

    // 1. Randomize Simulation Config
    sim::Simulation::Config sim_cfg;
    
    // Perturb inertia diagonal
    for (int i = 0; i < 3; ++i) {
        sim_cfg.inertia(i, i) *= std::max(0.5, norm(run_gen)); 
    }
    
    // Random initial tumble
    std::uniform_real_distribution<double> w_dist(-0.2, 0.2);
    sim_cfg.init_w = common::Vector3(w_dist(run_gen), w_dist(run_gen), w_dist(run_gen));

    sim::Simulation sim(sim_cfg);

    // 2. Setup FSW
    fsw::FlightSoftware::Config fsw_cfg;
    fsw_cfg.full_config = {
        {"fsw", {
            {"controllers", {
                {"attitude", {{"type", "PID"}, {"nominal_pid", {{"kp", 0.8}, {"ki", 0.05}, {"kd", 1.2}}}}},
                {"detumble", {{"type", "Bdot"}, {"gain", 50000.0}}}
            }},
            {"estimator", {{"type", "MEKF"}}}
        }}
    };
    fsw::FlightSoftware fsw(fsw_cfg);

    // 3. Inject random failures
    if (dist(run_gen) < config_.prob_dead_gyro) {
        sim.getGyro().injectFailure_Dead();
    }

    // 4. Run Simulation
    MonteCarloResult res;
    res.success = true;
    res.max_pointing_error_deg = 0.0;
    
    double dt = config_.dt;
    for (double t = 0; t < config_.duration; t += dt) {
        common::SensorData sensors = sim.getSensors();
        std::vector<std::vector<uint8_t>> empty_cmds;
        common::Vector3 torque = fsw.step(sensors, empty_cmds, dt);
        sim.step(dt, torque);

        // Track stats
        if (fsw.getCurrentMode() == fsw::core::MissionMode::NOMINAL) {
            common::Vector3 body_z = sim.getAttitude() * common::Vector3(0, 0, 1);
            common::Vector3 sun_inertial(1, 0, 0);
            double cos_theta = body_z.dot(sun_inertial);
            double error_deg = std::acos(std::clamp(cos_theta, -1.0, 1.0)) * common::RAD2DEG;
            res.max_pointing_error_deg = std::max(res.max_pointing_error_deg, error_deg);
        }

        // Failure if tumble rate explodes
        if (sim.getAngularRate().norm() > 1.0) {
            res.success = false;
            res.failure_reason = "Tumble rate limit exceeded";
            break;
        }
    }

    // 5. Final verification
    if (res.success) {
        res.final_rate_rads = sim.getAngularRate().norm();
        if (res.final_rate_rads > config_.pass_rate_rads * 5.0) { // Slack for MC
             res.success = false;
             res.failure_reason = "Final rate too high";
        }
        if (res.max_pointing_error_deg > config_.pass_pointing_error_deg * 5.0) {
             res.success = false;
             res.failure_reason = "Pointing error too high";
        }
    }

    return res;
}

} // namespace test
