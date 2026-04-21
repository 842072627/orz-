#pragma once

#include "Instance.h"
#include "PathTable.h"
#include "TimeSpaceAStar.h"
#include <vector>
#include <algorithm>
#include <random>
#include <chrono>

namespace LNS {

// LNS Neighbor - set of agents to replan together
struct Neighbor {
    std::vector<int> agents;  // agent ids
    bool succ = false;
    float sum_of_costs = 0;
    std::vector<std::vector<std::pair<int, int>>> paths;  // planned paths
};

// Simple LNS that uses TimeSpaceA* for replanning
class LNSWrapper {
public:
    Instance* instance;
    PathTable path_table;
    TimeSpaceAStar tsp;
    
    int neighbor_size = 5;  // Number of agents to replan
    int max_iterations = 10;
    int window_size = 10;
    
    // Current paths for all agents
    std::vector<std::vector<std::pair<int, int>>> current_paths;
    
    LNSWrapper() = default;
    
    void init(Instance* inst, int ws = 10) {
        instance = inst;
        window_size = ws;
        tsp = TimeSpaceAStar(inst, ws);
        
        // Initialize paths from current state
        current_paths.resize(instance->num_of_agents);
        for (int i = 0; i < instance->num_of_agents; i++) {
            current_paths[i].push_back({instance->start_locations[i], 0});
        }
        
        std::cout << "[LNS] Initialized with " << instance->num_of_agents 
                  << " agents, window_size=" << window_size << std::endl;
    }
    
    // Update PathTable from current paths
    void updatePathTable() {
        path_table.resize(instance->map_size, window_size);
        path_table.reset();
        
        for (int agent_id = 0; agent_id < (int)current_paths.size(); agent_id++) {
            if (!current_paths[agent_id].empty()) {
                path_table.insertPath(agent_id, current_paths[agent_id]);
            }
        }
    }
    
    // Generate neighbor: select random agents
    Neighbor generateNeighbor() {
        Neighbor neighbor;
        
        if (instance->num_of_agents <= neighbor_size) {
            // All agents
            neighbor.agents.resize(instance->num_of_agents);
            for (int i = 0; i < instance->num_of_agents; i++) {
                neighbor.agents[i] = i;
            }
        } else {
            // Random subset
            std::vector<int> indices(instance->num_of_agents);
            for (int i = 0; i < instance->num_of_agents; i++) indices[i] = i;
            std::random_shuffle(indices.begin(), indices.end());
            neighbor.agents.assign(indices.begin(), indices.begin() + neighbor_size);
        }
        
        return neighbor;
    }
    
    // Optimize neighbor using TimeSpaceA*
    bool optimize(Neighbor& neighbor, int time_limit_ms) {
        auto start_time = std::chrono::steady_clock::now();
        
        // Clear paths for neighbor agents
        neighbor.paths.resize(neighbor.agents.size());
        
        // Update path table with non-neighbor agents' paths
        updatePathTable();
        
        for (size_t i = 0; i < neighbor.agents.size(); i++) {
            int agent_id = neighbor.agents[i];
            
            // Check timeout
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > time_limit_ms) {
                return false;
            }
            
            // Plan path for this agent
            int start_loc = instance->start_locations[agent_id];
            int goal_loc = instance->goal_locations[agent_id];
            int start_time = 0;  // Start from current time
            
            // Find current progress for this agent
            if (!current_paths[agent_id].empty()) {
                // Continue from where agent currently is
                start_loc = current_paths[agent_id].back().first;
                start_time = current_paths[agent_id].size() - 1;
            }
            
            auto path = tsp.plan(agent_id, start_loc, goal_loc, start_time, path_table);
            
            if (path.empty()) {
                // Failed to find path
                neighbor.paths[i] = current_paths[agent_id];
            } else {
                neighbor.paths[i] = path;
            }
            
            neighbor.sum_of_costs += neighbor.paths[i].size();
        }
        
        neighbor.succ = true;
        return true;
    }
    
    // Run one iteration of LNS
    bool runIteration(int time_limit_ms = 100) {
        // 1. Generate neighbor
        Neighbor neighbor = generateNeighbor();
        
        // 2. Optimize neighbor
        if (!optimize(neighbor, time_limit_ms)) {
            return false;
        }
        
        // 3. Calculate cost improvement
        float old_cost = 0;
        for (int agent_id : neighbor.agents) {
            old_cost += current_paths[agent_id].size();
        }
        
        float new_cost = neighbor.sum_of_costs;
        
        // 4. Accept if improved
        if (new_cost < old_cost) {
            for (size_t i = 0; i < neighbor.agents.size(); i++) {
                int agent_id = neighbor.agents[i];
                current_paths[agent_id] = neighbor.paths[i];
            }
            std::cout << "[LNS] Improved: " << old_cost << " -> " << new_cost << std::endl;
            return true;
        }
        
        return false;
    }
    
    // Run full LNS
    void run(int time_limit_ms = 1000) {
        auto start_time = std::chrono::steady_clock::now();
        
        std::cout << "[LNS] Starting with " << instance->num_of_agents << " agents" << std::endl;
        
        int iterations = 0;
        while (iterations < max_iterations) {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start_time).count();
            if (elapsed > time_limit_ms) {
                std::cout << "[LNS] Timeout after " << iterations << " iterations" << std::endl;
                break;
            }
            
            int remaining_ms = time_limit_ms - (int)elapsed;
            bool improved = runIteration(remaining_ms);
            iterations++;
            
            if (!improved) {
                // No improvement, try again with different neighbor
            }
        }
        
        std::cout << "[LNS] Completed " << iterations << " iterations" << std::endl;
    }
    
    // Get current paths
    const std::vector<std::vector<std::pair<int, int>>>& getPaths() const {
        return current_paths;
    }
};

}
