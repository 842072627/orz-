#include "GlobalManager.h"

namespace LNS {

GlobalManager::GlobalManager(
    Instance& instance,
    MyPlanner::TrajLNS& lns,
    int num_threads,
    int max_iterations,
    int neighbor_size,
    destroy_heuristic destroy_strategy,
    bool ALNS,
    double decay_factor,
    double reaction_factor
) : instance(instance),
    lns(lns),
    num_threads(num_threads),
    max_iterations(max_iterations),
    neighbor_size(neighbor_size),
    destroy_strategy(destroy_strategy),
    ALNS(ALNS),
    decay_factor(decay_factor),
    reaction_factor(reaction_factor)
{
    // Create one NeighborGenerator and LocalOptimizer per thread
    for (int i = 0; i < num_threads; i++) {
        neighbor_generators.push_back(
            std::make_shared<NeighborGenerator>(
                instance, lns, neighbor_size, destroy_strategy,
                ALNS, decay_factor, reaction_factor
            )
        );
        
        local_optimizers.push_back(
            std::make_shared<LocalOptimizer>(
                instance, lns, "PP", neighbor_size
            )
        );
    }
}

void GlobalManager::reset() {
    for (auto& ng : neighbor_generators) {
        ng->reset();
    }
    for (auto& lo : local_optimizers) {
        lo->reset();
    }
}

void GlobalManager::run() {
    // Main LNS loop
    // 1. Generate neighbors
    // 2. Optimize (replan paths)
    // 3. Accept or reject
    
    for (int iter = 0; iter < max_iterations; iter++) {
        // Generate neighbor set
        Neighbor neighbor;
        for (int t = 0; t < num_threads; t++) {
            neighbor = neighbor_generators[t]->generate();
            if (!neighbor.agents.empty()) break;
        }
        
        if (neighbor.agents.empty()) continue;
        
        // Optimize
        local_optimizers[0]->optimize(neighbor);
        
        // Update LNS state if improvement
        if (neighbor.succ) {
            for (size_t i = 0; i < neighbor.agents.size(); i++) {
                int agent_id = neighbor.agents[i];
                if (agent_id < 0 || agent_id >= instance.num_of_agents) continue;
                
                // Update trajectory
                lns.trajs[agent_id] = neighbor.m_paths[i];
            }
            
            // Update ALNS weights
            if (ALNS) {
                for (auto& ng : neighbor_generators) {
                    ng->destroy_weights[neighbor.selected_neighbor] =
                        reaction_factor * (neighbor.old_sum_of_costs - neighbor.sum_of_costs) / neighbor.agents.size()
                        + (1 - reaction_factor) * ng->destroy_weights[neighbor.selected_neighbor];
                }
            }
        } else {
            // Decay weights on failure
            if (ALNS) {
                for (auto& ng : neighbor_generators) {
                    ng->destroy_weights[neighbor.selected_neighbor] =
                        (1 - decay_factor) * ng->destroy_weights[neighbor.selected_neighbor];
                }
            }
        }
    }
}

}
