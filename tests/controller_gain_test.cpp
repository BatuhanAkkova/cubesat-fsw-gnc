#include <iostream>
#include <vector>
#include "opt/GeneticOptimizer.hpp"
#include "test/MonteCarloRunner.hpp"

int main() {
    std::cout << "=== PHASE 8: ADVANCED CONTROL & OPTIMIZATION DEMO ===" << std::endl;

    // --- STEP 1: Parameter Optimization ---
    std::cout << "\n--- STEP 1: Optimizing PID Gains using Genetic Algorithm ---" << std::endl;
    
    opt::GeneticOptimizer::Config ga_cfg;
    ga_cfg.population_size = 20; // Smaller for demo speed
    ga_cfg.num_generations = 10;
    
    opt::GeneticOptimizer optimizer(ga_cfg);
    std::vector<double> best_gains = optimizer.optimize();

    std::cout << "\nOptimization Complete!" << std::endl;
    std::cout << "Best Gains Found: P=" << best_gains[0] << ", I=" << best_gains[1] << ", D=" << best_gains[2] << std::endl;

    // --- STEP 2: Robustness Analysis ---
    std::cout << "\n--- STEP 2: Running Robustness Analysis (Monte Carlo) ---" << std::endl;
    
    test::MonteCarloRunner::Config mc_cfg;
    mc_cfg.num_runs = 50; // Smaller for demo speed
    mc_cfg.duration = 40.0;
    
    test::MonteCarloRunner mc_runner(mc_cfg);
    mc_runner.run();

    std::cout << "\nDemo Finished." << std::endl;
    return 0;
}
