#include "NeighborGenerator.h"
#include <algorithm>
#include <cstdlib>

namespace LNS {

NeighborGenerator::NeighborGenerator(
    Instance& instance,
    TrajLNS& lns,
    int neighbor_size,
    destroy_heuristic destroy_strategy,
    bool ALNS,
    double decay_factor,
    double reaction_factor
) : instance(instance),
       lns(lns),
       neighbor_size(neighbor_size),
       destroy_strategy(destroy_strategy),
       ALNS(ALNS),
       decay_factor(decay_factor),
       reaction_factor(reaction_factor),
       MT(0)  // random seed
{
    destroy_weights.assign(COUNT, 1.0);
    
    // Find intersections (locations with degree > 2)
    for (int i = 0; i < instance.map_size; i++) {
        if (!instance.isObstacle(i) && getDegree(i) > 2) {
            intersections.push_back(i);
        }
    }
}

void NeighborGenerator::reset() {
    destroy_weights.assign(COUNT, 1.0);
}

int NeighborGenerator::findMostDelayedAgent() {
    int most_delayed_agent = -1;
    int max_delay = 0;
    
    for (int i = 0; i < instance.num_of_agents; i++) {
        if (lns.trajs[i].empty()) continue;
        
        int delay = calculateDelay(i);
        if (delay > max_delay) {
            max_delay = delay;
            most_delayed_agent = i;
        }
    }
    
    return most_delayed_agent;
}

double NeighborGenerator::calculateDelay(int agent_id) {
    // Calculate how far behind schedule the agent is
    // Based on: expected time vs actual progress
    auto& traj = lns.trajs[agent_id];
    if (traj.empty()) return 0;
    
    // Simple delay metric: difference between expected and actual progress
    int curr_loc = instance.start_locations[agent_id];
    int goal_loc = instance.goal_locations[agent_id];
    int expected_dist = instance.getManhattanDistance(curr_loc, goal_loc);
    int actual_dist = (int)traj.size() - 1;
    
    return expected_dist - actual_dist;
}

void NeighborGenerator::randomWalk(int start_agent, int start_time, std::vector<int>& neighbors) {
    if (start_agent < 0 || start_agent >= instance.num_of_agents) return;
    
    auto& start_traj = lns.trajs[start_agent];
    if (start_traj.empty()) return;
    
    int current_loc;
    if (start_time < (int)start_traj.size()) {
        current_loc = start_traj[start_time];
    } else {
        current_loc = start_traj.back();
    }
    
    neighbors.push_back(start_agent);
    
    // Random walk for a few steps
    int walk_steps = 5 + rand() % 10;
    for (int step = 0; step < walk_steps && (int)neighbors.size() < neighbor_size; step++) {
        auto nbrs = instance.getNeighbors(current_loc);
        if (nbrs.empty()) break;
        
        int next_idx = rand() % nbrs.size();
        current_loc = nbrs[next_idx];
        
        // Find which agent is at this location
        for (int i = 0; i < instance.num_of_agents; i++) {
            if (i == start_agent) continue;
            if (lns.trajs[i].empty()) continue;
            
            // Check if agent i passes through current_loc
            for (int t = 0; t < (int)lns.trajs[i].size(); t++) {
                if (lns.trajs[i][t] == current_loc) {
                    // Check if not already in neighbors
                    bool found = false;
                    for (int n : neighbors) {
                        if (n == i) { found = true; break; }
                    }
                    if (!found) {
                        neighbors.push_back(i);
                        if ((int)neighbors.size() >= neighbor_size) return;
                    }
                    break;
                }
            }
        }
    }
}

Neighbor NeighborGenerator::generate() {
    Neighbor neighbor;
    
    switch (destroy_strategy) {
        case RANDOMWALK: {
            // Find most delayed agent and do random walk
            int a = findMostDelayedAgent();
            if (a < 0) return neighbor;
            
            randomWalk(a, 0, neighbor.agents);
            
            if ((int)neighbor.agents.size() < 2) return neighbor;
            
            // Shuffle to make it random
            std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
            neighbor.selected_neighbor = RANDOMWALK;
            break;
        }
        
        case RANDOM: {
            // Randomly select agents
            for (int i = 0; i < instance.num_of_agents; i++)
                neighbor.agents.push_back(i);
            std::random_shuffle(neighbor.agents.begin(), neighbor.agents.end());
            if ((int)neighbor.agents.size() > neighbor_size)
                neighbor.agents.resize(neighbor_size);
            neighbor.selected_neighbor = RANDOM;
            break;
        }
        
        default:
            break;
    }
    
    return neighbor;
}

}
