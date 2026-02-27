#include <iostream>
#include <vector>
#include <gtest/gtest.h>
#include "opt/GeneticOptimizer.hpp"
#include "test/MonteCarloRunner.hpp"

TEST(ControllerGainTest, OptimizationAndRobustness) {
    std::cout << "=== PHASE 8: ADVANCED CONTROL & OPTIMIZATION DEMO ===" << std::endl;

    // --- STEP 1: Parameter Optimization ---
    std::cout << "\n--- STEP 1: Optimizing PID Gains using Genetic Algorithm ---" << std::endl;
    
    opt::GeneticOptimizer::Config ga_cfg;
    ga_cfg.population_size = 10; // Reduced for test speed
    ga_cfg.num_generations = 5;
    
    opt::GeneticOptimizer optimizer(ga_cfg);
    std::vector<double> best_gains = optimizer.optimize();

    std::cout << "\nOptimization Complete!" << std::endl;
    std::cout << "Best Gains Found: P=" << best_gains[0] << ", I=" << best_gains[1] << ", D=" << best_gains[2] << std::endl;
    
    ASSERT_EQ(best_gains.size(), 3);

    // --- STEP 2: Robustness Analysis ---
    std::cout << "\n--- STEP 2: Running Robustness Analysis (Monte Carlo) ---" << std::endl;
    
    test::MonteCarloRunner::Config mc_cfg;
    mc_cfg.num_runs = 10; // Reduced for test speed
    mc_cfg.duration = 10.0;
    
    test::MonteCarloRunner mc_runner(mc_cfg);
    mc_runner.run();

    std::cout << "\nDemo Finished." << std::endl;
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
