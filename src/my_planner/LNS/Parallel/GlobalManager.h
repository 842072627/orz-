#pragma once
#include <memory>
#include <vector>
#include <string>
#include "../Instance.h"
#include "NeighborGenerator.h"
#include "LocalOptimizer.h"

namespace LNS {

class GlobalManager {
public:
    Instance& instance;
    TrajLNS& lns;
    
    int num_threads;
    int max_iterations;
    int neighbor_size;
    destroy_heuristic destroy_strategy;
    bool ALNS;
    double decay_factor;
    double reaction_factor;
    
    std::vector<std::shared_ptr<NeighborGenerator>> neighbor_generators;
    std::vector<std::shared_ptr<LocalOptimizer>> local_optimizers;
    
    GlobalManager(
        Instance& instance,
        TrajLNS& lns,
        int num_threads,
        int max_iterations,
        int neighbor_size,
        destroy_heuristic destroy_strategy,
        bool ALNS,
        double decay_factor,
        double reaction_factor
    );
    
    void reset();
    void run();
};

}
