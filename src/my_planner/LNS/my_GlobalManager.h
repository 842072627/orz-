#pragma once
#include <memory>
#include <chrono>
#include <unordered_set>
#include "my_PathTable.h"
#include "my_NeighborGenerator.h"
#include "my_LocalOptimizer.h"
#include "my_Instance.h"
#include "my_TimeSpaceAStarPlanner.h"
#include "../my_Types.h"

namespace LNS {

// Minimal GlobalManager for LNS coordination
class GlobalManager
{
public:
    // Configuration
    int num_of_agents;
    int window_size_for_PATH;
    int neighbor_size;
    
    // Components
    std::shared_ptr<Instance> instance;
    PathTable path_table;
    NeighborGenerator neighbor_generator;
    LocalOptimizer local_optimizer;
    
    // State
    std::vector<std::vector<std::pair<int, int>>> paths;  // Current paths for all agents
    std::vector<int> goal_locs;  // Goal locations for all agents
    float sum_of_costs = 0;
    int num_failures = 0;
    
    // Time tracking
    using TimePoint = std::chrono::steady_clock::time_point;
    
    GlobalManager() = default;
    
    // Initialize with problem instance
    GlobalManager(
        const std::vector<int>& map,
        int num_rows,
        int num_cols,
        int num_agents,
        const std::vector<int>& start_locs,
        const std::vector<int>& start_orients,
        const std::vector<int>& goal_locs,
        int window_size = 10,
        int neighbor_size = 8
    ) : num_of_agents(num_agents),
        window_size_for_PATH(window_size),
        neighbor_size(neighbor_size),
        goal_locs(goal_locs)
    {
        // Create instance
        instance = std::make_shared<Instance>();
        instance->num_of_rows = num_rows;
        instance->num_of_cols = num_cols;
        instance->map_size = map.size();
        instance->my_map = map;
        instance->num_of_agents = num_of_agents;
        instance->start_locations = start_locs;
        instance->start_orientations = start_orients;
        instance->goal_locations = goal_locs;
        
        // Initialize path table
        path_table.resize(map.size(), window_size + 1);
        
        // Initialize components
        neighbor_generator = NeighborGenerator(num_agents, neighbor_size, 0);
        local_optimizer = LocalOptimizer(num_agents, window_size, 0);
        local_optimizer.init(instance, goal_locs);
        
        // Initialize paths
        paths.resize(num_agents);
    }
    
    // Run LNS main loop
    // Returns: actions for next step
    std::vector<int> run(double time_limit_ms) {
        TimePoint start = std::chrono::steady_clock::now();
        TimePoint end = start + std::chrono::milliseconds((int)time_limit_ms);
        
        // Main LNS loop
        while (std::chrono::steady_clock::now() < end) {
            // 1. Generate neighbors (destroy some agents' paths)
            std::unordered_set<int> destroyed;
            neighbor_generator.generateRandomWalk(destroyed);
            
            if (destroyed.empty()) break;
            
            // 2. Replan paths for destroyed agents
            std::vector<std::vector<std::pair<int, int>>> old_paths = paths;
            bool improved = local_optimizer.optimize(destroyed, paths, path_table, 100.0);
            
            // 3. Accept/reject
            if (!improved) {
                paths = old_paths;  // Revert
                num_failures++;
            }
            
            // Check convergence
            if (num_failures > 10) break;
        }
        
        // Extract actions from paths
        return getActions();
    }
    
    // Get actions for next step from paths
    std::vector<int> getActions() {
        std::vector<int> actions(num_of_agents, 3);  // Default: W (wait)
        
        for (int i = 0; i < num_of_agents; ++i) {
            if (paths[i].size() >= 2) {
                int curr_loc = paths[i][0].first;
                int curr_orient = paths[i][0].second;
                int next_loc = paths[i][1].first;
                int next_orient = paths[i][1].second;
                
                // Compute action from current to next
                actions[i] = computeAction(curr_loc, curr_orient, next_loc, next_orient);
            }
        }
        
        return actions;
    }
    
    // Compute action code from state transition
    int computeAction(int curr_loc, int curr_orient, int next_loc, int next_orient) {
        if (next_loc == curr_loc) {
            // Wait or turn
            int orient_diff = (next_orient - curr_orient + 4) % 4;
            if (orient_diff == 1) return 1;  // CR
            if (orient_diff == 3) return 2; // CCR
            return 3;  // W
        } else {
            return 0;  // FW
        }
    }
    
    // Reset manager state
    void reset() {
        path_table.reset();
        sum_of_costs = 0;
        num_failures = 0;
    }
    
    // Update with new problem state
    void update(
        const std::vector<int>& curr_locs,
        const std::vector<int>& curr_orients,
        const std::vector<int>& goal_locs_new
    ) {
        instance->start_locations = curr_locs;
        instance->start_orientations = curr_orients;
        instance->goal_locations = goal_locs_new;
        goal_locs = goal_locs_new;
        local_optimizer.updateGoals(goal_locs_new);
        
        // Update paths to start from current locations
        for (int i = 0; i < num_of_agents; ++i) {
            if (!paths[i].empty() && paths[i][0].first != curr_locs[i]) {
                // Agent has moved, shift path
                paths[i].insert(paths[i].begin(), {curr_locs[i], curr_orients[i]});
                if (paths[i].size() > (size_t)window_size_for_PATH + 1) {
                    paths[i].pop_back();
                }
            }
        }
    }
};

} // namespace LNS
