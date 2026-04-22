#pragma once
#include <vector>
#include <queue>
#include <unordered_set>
#include <chrono>
#include <random>
#include "../my_planner/my_Types.h"
#include "../my_search_node.h"
#include "../my_heuristics.h"
#include "my_Instance.h"
#include "my_PathTable.h"

namespace LNS {

// Time-Space A* for LNS replanning
// Considers temporal constraints from PathTable
class TimeSpaceAStar {
public:
    int num_of_agents;
    Instance* instance;
    int window_size;
    
    // Search configuration
    int max_expansions = 10000;
    bool use_path_table = true;
    
    TimeSpaceAStar() = default;
    
    TimeSpaceAStar(Instance* inst, int ws = 10) 
        : instance(inst), window_size(ws) {}
    
    // Plan path for one agent considering PathTable constraints
    // agent_id: which agent we're planning for (to avoid its own path)
    // start: starting location
    // goal: goal location  
    // start_time: earliest timestep to start
    // path_table: other agents' paths to avoid
    // Returns: vector of (location, orientation) pairs
    std::vector<std::pair<int, int>> plan(
        int agent_id,
        int start,
        int goal,
        int start_time,
        const PathTable& path_table
    ) {
        std::vector<std::pair<int, int>> path;
        
        if (!instance || start < 0 || goal < 0) {
            return path;
        }
        
        if (start == goal) {
            path.push_back({start, 0});
            return path;
        }
        
        // Simple greedy search with conflict avoidance
        int curr = start;
        int curr_time = start_time;
        int orientation = 0;
        
        path.push_back({curr, orientation});
        
        int steps = 0;
        int max_steps = window_size * 10;  // Reasonable horizon
        
        while (curr != goal && steps < max_steps) {
            steps++;
            
            // Get neighbors
            int neighbors[4];
            neighbors[0] = curr + 1;           // East
            neighbors[1] = curr + instance->num_of_cols; // South
            neighbors[2] = curr - 1;           // West
            neighbors[3] = curr - instance->num_of_cols; // North
            
            int best_next = -1;
            int best_score = INT_MAX;
            int best_orientation = orientation;
            
            for (int i = 0; i < 4; i++) {
                int next = neighbors[i];
                if (next < 0 || next >= instance->map_size) continue;
                if (instance->isObstacle(next)) continue;
                
                // Check if edge is blocked at curr_time
                // Edge conflict: if another agent is at 'next' at curr_time+1
                // and came from a different location
                if (use_path_table && path_table.edgeConstrained(curr, next, curr_time + 1)) {
                    continue;
                }
                
                // Check if location is blocked at curr_time
                if (use_path_table && path_table.constrained(next, curr_time + 1)) {
                    continue;
                }
                
                // Heuristic: Manhattan distance to goal
                int h = manhattanDistance(next, goal, instance->num_of_cols);
                
                // Prefer forward movement
                int score = h;
                if (i == orientation) score -= 1;  // Forward bonus
                
                if (score < best_score) {
                    best_score = score;
                    best_next = next;
                    best_orientation = i;
                }
            }
            
            if (best_next == -1) {
                // Dead end - wait
                curr_time++;
                path.push_back({curr, orientation});
                if (curr_time - start_time > max_steps) break;
                continue;
            }
            
            curr = best_next;
            orientation = best_orientation;
            curr_time++;
            path.push_back({curr, orientation});
            
            if (curr == goal) break;
        }
        
        return path;
    }
    
    // Check if a move is valid (within bounds and not obstacle)
    bool isValidMove(int from, int to) {
        if (to < 0 || to >= instance->map_size) return false;
        if (instance->isObstacle(to)) return false;
        
        // Check if adjacent
        int diff = to - from;
        if (diff == 1 || diff == -1 || diff == instance->num_of_cols || diff == -instance->num_of_cols) {
            return true;
        }
        return false;
    }
    
    // Manhattan distance heuristic
    int manhattanDistance(int loc1, int loc2, int cols) {
        int r1 = loc1 / cols;
        int c1 = loc1 % cols;
        int r2 = loc2 / cols;
        int c2 = loc2 % cols;
        return abs(r1 - r2) + abs(c1 - c2);
    }
};

}
