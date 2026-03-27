#include "opt/GeneticOptimizer.hpp"

#include <chrono>

#include "common/logger.hpp"

namespace opt {

GeneticOptimizer::GeneticOptimizer(const Config& config)
    : config_(config), gen_(std::chrono::system_clock::now().time_since_epoch().count()) {}

std::vector<double> GeneticOptimizer::optimize() {
    initializePopulation();

    for (int g = 0; g < config_.num_generations; ++g) {
        evaluatePopulation();

        // Sort by cost (ascending)
        std::sort(population_.begin(), population_.end(),
                  [](const Individual& a, const Individual& b) { return a.cost < b.cost; });

        common::LogInfo("Generation {} | Best Cost: {:.4f} | Gains: [{:.2f}, {:.2f}, {:.2f}]", g, population_[0].cost,
                        population_[0].genes[0], population_[0].genes[1], population_[0].genes[2]);

        if (g < config_.num_generations - 1) {
            evolve();
        }
    }

    return population_[0].genes;
}

void GeneticOptimizer::initializePopulation() {
    population_.clear();
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < config_.population_size; ++i) {
        Individual ind;
        for (size_t j = 0; j < config_.min_gains.size(); ++j) {
            double range = config_.max_gains[j] - config_.min_gains[j];
            ind.genes.push_back(config_.min_gains[j] + dist(gen_) * range);
        }
        population_.push_back(ind);
    }
}

void GeneticOptimizer::evaluatePopulation() {
    for (auto& ind : population_) {
        auto res = evaluator_.evaluate(ind.genes);
        ind.cost = res.cost;
    }
}

void GeneticOptimizer::evolve() {
    std::vector<Individual> next_gen;

    // 1. Keep elites
    for (int i = 0; i < config_.num_elites; ++i) {
        next_gen.push_back(population_[i]);
    }

    // 2. Breed the rest
    std::uniform_int_distribution<int> parent_dist(0, config_.num_elites * 2);  // Sample from top performers

    while (next_gen.size() < (size_t)config_.population_size) {
        int idx1 = parent_dist(gen_) % population_.size();
        int idx2 = parent_dist(gen_) % population_.size();

        Individual child = breed(population_[idx1], population_[idx2]);
        mutate(child);
        next_gen.push_back(child);
    }

    population_ = next_gen;
}

Individual GeneticOptimizer::breed(const Individual& parent1, const Individual& parent2) {
    Individual child;
    std::uniform_real_distribution<double> coin(0.0, 1.0);

    for (size_t i = 0; i < parent1.genes.size(); ++i) {
        // Simple blend/average crossover
        double weight = coin(gen_);
        child.genes.push_back(weight * parent1.genes[i] + (1.0 - weight) * parent2.genes[i]);
    }
    return child;
}

void GeneticOptimizer::mutate(Individual& ind) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    std::normal_distribution<double> step(0.0, config_.mutation_step);

    for (size_t i = 0; i < ind.genes.size(); ++i) {
        if (coin(gen_) < config_.mutation_rate) {
            ind.genes[i] += step(gen_);
            // Clamp to bounds
            ind.genes[i] = std::max(config_.min_gains[i], std::min(config_.max_gains[i], ind.genes[i]));
        }
    }
}

}  // namespace opt
