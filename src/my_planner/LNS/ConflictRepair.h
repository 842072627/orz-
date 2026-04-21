// ConflictRepair - Simple post-processing to fix guide path conflicts
// Doesn't require full constraint-based A*, just local repairs

#pragma once

#include <vector>
#include <unordered_set>
#include <algorithm>

namespace LNS {

// Simple conflict detection and repair for guide paths
class ConflictRepair {
public:
    int map_rows;
    int map_cols;
    int max_timestep;
    
    ConflictRepair(int rows, int cols, int max_ts = 100) 
        : map_rows(rows), map_cols(cols), max_timestep(max_ts) {}
    
    // Check if two paths conflict (vertex or edge)
    static bool hasConflict(const std::vector<int>& path1, const std::vector<int>& path2, int t1, int t2) {
        int len1 = path1.size();
        int len2 = path2.size();
        
        // Check vertex conflicts
        for (int dt = 0; dt < 10 && t1 + dt < len1 && t2 + dt < len2; dt++) {
            if (path1[t1 + dt] == path2[t2 + dt]) {
                return true;  // Vertex conflict
            }
            // Edge conflict (swapping positions)
            if (t1 + dt + 1 < len1 && t2 + dt + 1 < len2 &&
                path1[t1 + dt] == path2[t2 + dt + 1] &&
                path1[t1 + dt + 1] == path2[t2 + dt]) {
                return true;  // Edge conflict (swap)
            }
        }
        return false;
    }
    
    // Repair a path by avoiding conflicts with other paths
    std::vector<int> repairPath(const std::vector<int>& path, 
                                  int start_idx,
                                  const std::vector<std::vector<int>>& other_paths,
                                  const std::vector<int>& start_times,
                                  int goal,
                                  int max_repair_steps = 20) {
        if (path.size() <= (size_t)start_idx) {
            return path;
        }
        
        std::vector<int> new_path(path.begin(), path.begin() + start_idx);
        int current_loc = new_path.back();
        int current_time = start_idx;
        
        // Simple greedy repair
        for (int step = 0; step < max_repair_steps && current_loc != goal; step++) {
            // Get neighbors (4-directional)
            std::vector<int> neighbors;
            int r = current_loc / map_cols;
            int c = current_loc % map_cols;
            
            if (r > 0) neighbors.push_back(current_loc - map_cols);  // up
            if (r < map_rows - 1) neighbors.push_back(current_loc + map_cols);  // down
            if (c > 0) neighbors.push_back(current_loc - 1);  // left
            if (c < map_cols - 1) neighbors.push_back(current_loc + 1);  // right
            
            // Sort by distance to goal
            std::sort(neighbors.begin(), neighbors.end(), [&](int a, int b) {
                int ra = a / map_cols, ca = a % map_cols;
                int rb = b / map_cols, cb = b % map_cols;
                int rg = goal / map_cols, cg = goal % map_cols;
                return (std::abs(ra - rg) + std::abs(ca - cg)) < 
                       (std::abs(rb - rg) + std::abs(cb - cg));
            });
            
            // Add current location as last resort
            neighbors.push_back(current_loc);
            
            // Try each neighbor until find one without conflict
            bool found = false;
            for (int next : neighbors) {
                if (next == current_loc) {
                    // Wait action - check if it causes conflict
                    bool conflict = false;
                    for (size_t i = 0; i < other_paths.size(); i++) {
                        int t_other = current_time - start_times[i];
                        if (t_other >= 0 && t_other < (int)other_paths[i].size()) {
                            if (other_paths[i][t_other] == current_loc) {
                                conflict = true;
                                break;
                            }
                        }
                    }
                    if (!conflict) {
                        new_path.push_back(next);
                        found = true;
                        break;
                    }
                } else {
                    // Move action - check if it causes edge conflict
                    bool conflict = false;
                    for (size_t i = 0; i < other_paths.size(); i++) {
                        int t_other = current_time - start_times[i];
                        if (t_other >= 0 && t_other + 1 < (int)other_paths[i].size()) {
                            // Edge conflict check
                            if (other_paths[i][t_other] == next && 
                                other_paths[i][t_other + 1] == current_loc) {
                                conflict = true;
                                break;
                            }
                        }
                    }
                    if (!conflict) {
                        new_path.push_back(next);
                        current_loc = next;
                        found = true;
                        break;
                    }
                }
            }
            
            if (!found) {
                // No valid move, wait
                new_path.push_back(current_loc);
            }
            current_time++;
        }
        
        // Append remaining original path
        for (size_t i = start_idx + 1; i < path.size(); i++) {
            new_path.push_back(path[i]);
        }
        
        return new_path;
    }
    
    // Repair all paths in a neighborhood
    void repairNeighbor(std::vector<std::vector<int>>& paths,
                        const std::vector<int>& agent_ids,
                        const std::vector<int>& goals,
                        int window_start = 0) {
        if (agent_ids.empty()) return;
        
        // For each agent, check conflicts and repair if needed
        for (size_t i = 0; i < agent_ids.size(); i++) {
            int agent_id = agent_ids[i];
            if ((int)paths.size() <= agent_id) continue;
            
            std::vector<int>& path = paths[agent_id];
            if (path.empty()) continue;
            
            // Check conflicts with other agents
            bool has_conflict = false;
            for (size_t j = 0; j < agent_ids.size(); j++) {
                if (i == j) continue;
                int other_id = agent_ids[j];
                if ((int)paths.size() <= other_id) continue;
                
                if (hasConflict(path, paths[other_id], window_start, window_start)) {
                    has_conflict = true;
                    break;
                }
            }
            
            if (has_conflict) {
                // Build list of other paths
                std::vector<std::vector<int>> other_paths;
                std::vector<int> start_times;
                for (size_t j = 0; j < agent_ids.size(); j++) {
                    if (i == j) continue;
                    int other_id = agent_ids[j];
                    if ((int)paths.size() > other_id && !paths[other_id].empty()) {
                        other_paths.push_back(paths[other_id]);
                        start_times.push_back(window_start);
                    }
                }
                
                // Repair this path
                path = repairPath(path, window_start, other_paths, start_times, goals[i]);
            }
        }
    }
};

}
