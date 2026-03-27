#pragma once

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

#include "opt/FitnessEvaluator.hpp"

namespace opt {

struct Individual {
    std::vector<double> genes;  // [kp, ki, kd]
    double cost = 1e12;
};

class GeneticOptimizer {
   public:
    struct Config {
        int population_size = 50;
        int num_generations = 50;
        double mutation_rate = 0.2;
        double mutation_step = 0.1;
        int num_elites = 5;

        std::vector<double> min_gains = {0.0, 0.0, 0.0};
        std::vector<double> max_gains = {5.0, 0.5, 10.0};
    };

    GeneticOptimizer(const Config& config);

    /**
     * @brief Run the optimization process.
     * @return Best set of gains found.
     */
    std::vector<double> optimize();

   private:
    void initializePopulation();
    void evaluatePopulation();
    void evolve();
    Individual breed(const Individual& parent1, const Individual& parent2);
    void mutate(Individual& ind);

    Config config_;
    std::vector<Individual> population_;
    FitnessEvaluator evaluator_;

    std::mt19937 gen_;
};

}  // namespace opt
