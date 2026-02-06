#pragma once

#include <vector>
#include <random>
#include <iostream>
#include "sim/Simulation.hpp"
#include "fsw/FlightSoftware.hpp"

namespace test {

struct MonteCarloResult {
    bool success;
    double max_pointing_error_deg;
    double final_rate_rads;
    std::string failure_reason;
};

class MonteCarloRunner {
public:
    struct Config {
        int num_runs = 100;
        double duration = 100.0;
        double dt = 0.1;
        
        // Randomization ranges
        double inertia_variation = 0.2; // +/- 20%
        double gyro_noise_max = 0.01;
        double mag_noise_max = 1e-6;
        
        // Failure probabilities
        double prob_dead_gyro = 0.05;
        double prob_dead_wheel = 0.05;
        
        double pass_pointing_error_deg = 10.0;
        double pass_rate_rads = 0.02;
    };

    MonteCarloRunner(const Config& config);

    void run();

private:
    MonteCarloResult runSingleSimulation(int seed);

    Config config_;
    std::mt19937 gen_;
};

} // namespace test
