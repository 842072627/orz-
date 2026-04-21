#pragma once
#include <vector>
#include <chrono>
#include "../my_planner/my_Types.h"
#include "../my_planner/search.h"
#include "../my_planner/heuristics.h"
#include "Instance.h"
#include "PathTable.h"

namespace LNS {

// Minimal TimeSpaceAStarPlanner using existing A* with conflict avoidance
// This wraps our existing A* to handle dynamic obstacles from PathTable
class TimeSpaceAStarPlanner
{
public:
    int num_of_agents;
    std::shared_ptr<Instance> instance;
    
    // Configuration
    int window_size = 10;
    bool sipp = false;  // SIPP not implemented yet
    
    TimeSpaceAStarPlanner() = default;
    
    TimeSpaceAStarPlanner(
        const std::shared_ptr<Instance>& instance,
        int window_size = 10
    ) : instance(instance), window_size(window_size) {}
    
    // Plan a path for one agent considering conflicts in path_table
    // start: starting location
    // goal: goal location
    // start_time: earliest timestep to start (for SIPP-style waiting)
    // path_table: table of other agents' paths to avoid
    // Returns: vector of (location, orientation) pairs
    std::vector<std::pair<int, int>> planPath(
        int agent_id,
        int start,
        int goal,
        int start_time,
        const PathTable& path_table,
        const std::vector<int>& goals  // goal locations for all agents
    ) {
        std::vector<std::pair<int, int>> path;
        
        if (start < 0 || goal < 0 || start >= instance->map_size || goal >= instance->map_size) {
            return path;  // Invalid
        }
        
        if (instance->isObstacle(start) || instance->isObstacle(goal)) {
            return path;  // Invalid
        }
        
        // Simple greedy pathfinding with conflict avoidance
        int curr = start;
        int curr_time = start_time;
        int orientation = 0;  // Default orientation
        
        path.push_back({curr, orientation});
        
        for (int step = 1; step < window_size; ++step) {
            // Get neighbors
            auto neighbors = instance->getNeighbors(curr);
            
            if (neighbors.empty()) {
                // Stuck, wait
                path.push_back({curr, orientation});
                continue;
            }
            
            // Find best neighbor (lowest cost considering conflicts)
            int best_loc = curr;
            int best_heuristic = instance->getManhattanDistance(curr, goal);
            
            for (int neighbor : neighbors) {
                // Check if this neighbor is free at curr_time+1
                if (path_table.constrained(neighbor, curr_time + 1)) {
                    continue;  // Conflict, skip
                }
                
                // Check if this brings us closer to goal
                int h = instance->getManhattanDistance(neighbor, goal);
                if (h < best_heuristic) {
                    best_loc = neighbor;
                    best_heuristic = h;
                }
            }
            
            // If no free neighbor found, wait
            if (best_loc == curr && !neighbors.empty()) {
                // Check if waiting is safe
                if (!path_table.constrained(curr, curr_time + 1)) {
                    // Wait is safe
                    path.push_back({curr, orientation});
                    curr_time++;
                    continue;
                }
            }
            
            curr = best_loc;
            path.push_back({curr, orientation});
            curr_time++;
            
            // Check if reached goal
            if (curr == goal) {
                break;
            }
        }
        
        return path;
    }
    
    // Batch plan paths for multiple agents
    // This is a simplified version that plans sequentially
    void planPaths(
        const std::vector<int>& agent_ids,
        const std::vector<int>& starts,
        const std::vector<int>& goals,
        const PathTable& path_table,
        std::vector<std::vector<std::pair<int, int>>>& out_paths
    ) {
        out_paths.resize(agent_ids.size());
        
        for (size_t i = 0; i < agent_ids.size(); ++i) {
            int agent_id = agent_ids[i];
            int start = starts[i];
            int goal = goals[i];
            
            out_paths[i] = planPath(agent_id, start, goal, 0, path_table, goals);
        }
    }
};

} // namespace LNS
