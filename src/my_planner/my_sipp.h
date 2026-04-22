// A138: Minimal SIPP implementation
// Uses ReservationTable to avoid temporal conflicts

#pragma once

#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <algorithm>
#include "my_reservation.h"

namespace MyPlanner {

// SIPP Node for TimeSpace A*
struct SIPPNode {
    int pos;       // position
    int timestep;  // time step
    int g;         // cost so far
    int h;         // heuristic
    int f() const { return g + h; }
    SIPPNode* parent;
    
    SIPPNode(int p, int t, int g, int h, SIPPNode* par) 
        : pos(p), timestep(t), g(g), h(h), parent(par) {}
};

// Priority queue comparison (min-heap by f)
struct SIPPCompare {
    bool operator()(const SIPPNode* a, const SIPPNode* b) const {
        if (a->f() == b->f()) {
            return a->g <= b->g;  // break ties by larger g
        }
        return a->f() >= b->f();  // min-heap
    }
};

// Find path using SIPP (Safe Interval Path Planning)
// Returns true if path found, fills path vector with positions
bool findPathSIPP(
    SharedEnvironment* env,
    int start, 
    int goal, 
    ReservationTable& rt,
    std::vector<int>& path,
    int max_timestep = 1000
) {
    if (start == goal) {
        path.clear();
        path.push_back(start);
        return true;
    }
    
    // Priority queue for A*
    std::priority_queue<SIPPNode*, std::vector<SIPPNode*>, SIPPCompare> open;
    
    // Track generated states: (pos, timestep) -> node
    std::unordered_set<int> closed;  // encoded as pos * 10000 + timestep
    
    // Helper to encode state
    auto encode = [](int pos, int t) { return pos * 10000 + t; };
    
    // Manhattan heuristic
    auto heuristic = [env](int p, int g) {
        int px = p % env->cols;
        int py = p / env->cols;
        int gx = g % env->cols;
        int gy = g / env->cols;
        return std::abs(px - gx) + std::abs(py - gy);
    };
    
    // Get neighbors (4-connected grid)
    auto get_neighbors = [env](int p, std::vector<int>& neighbors) {
        neighbors.clear();
        int cols = env->cols;
        int rows = env->map.size() / cols;
        int x = p % cols;
        int y = p / cols;
        
        if (x > 0 && env->map[p - 1] == 0) neighbors.push_back(p - 1);  // left
        if (x < cols - 1 && env->map[p + 1] == 0) neighbors.push_back(p + 1);  // right
        if (y > 0 && env->map[p - cols] == 0) neighbors.push_back(p - cols);  // up
        if (y < rows - 1 && env->map[p + cols] == 0) neighbors.push_back(p + cols);  // down
    };
    
    // Create start node
    int start_h = heuristic(start, goal);
    SIPPNode* start_node = new SIPPNode(start, 0, 0, start_h, nullptr);
    open.push(start_node);
    
    while (!open.empty()) {
        SIPPNode* curr = open.top();
        open.pop();
        
        // Check if already expanded
        int curr_encoded = encode(curr->pos, curr->timestep);
        if (closed.find(curr_encoded) != closed.end()) {
            delete curr;
            continue;
        }
        closed.insert(curr_encoded);
        
        // Check if reached goal
        if (curr->pos == goal) {
            // Reconstruct path
            path.clear();
            SIPPNode* node = curr;
            while (node != nullptr) {
                path.push_back(node->pos);
                node = node->parent;
            }
            std::reverse(path.begin(), path.end());
            
            // Cleanup
            while (!open.empty()) {
                delete open.top();
                open.pop();
            }
            return true;
        }
        
        // Check timestep limit
        if (curr->timestep >= max_timestep) {
            continue;
        }
        
        // Get neighbors
        std::vector<int> neighbors;
        get_neighbors(curr->pos, neighbors);
        
        // Try waiting at current position
        neighbors.push_back(curr->pos);
        
        for (int next_pos : neighbors) {
            int next_t = curr->timestep + 1;
            int next_encoded = encode(next_pos, next_t);
            
            // Skip if already expanded
            if (closed.find(next_encoded) != closed.end()) {
                continue;
            }
            
            // Check ReservationTable: is this position free at this time?
            if (!rt.isFree(next_pos, next_t)) {
                continue;  // Position is occupied, skip this successor
            }
            
            // Calculate cost and heuristic
            int move_cost = (next_pos == curr->pos) ? 1 : 1;  // wait or move
            int g = curr->g + move_cost;
            int h = heuristic(next_pos, goal);
            
            // Create successor node
            SIPPNode* next_node = new SIPPNode(next_pos, next_t, g, h, curr);
            open.push(next_node);
        }
    }
    
    return false;  // No path found
}

}  // namespace MyPlanner