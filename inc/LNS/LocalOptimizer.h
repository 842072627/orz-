#pragma once
#include <random>
#include <vector>
#include <unordered_set>
#include "PathTable.h"
#include "Instance.h"
#include "TimeSpaceAStarPlanner.h"

namespace LNS {

// Minimal LocalOptimizer for replanning paths
// Uses TimeSpaceA* to replan paths for destroyed agents with conflict avoidance
class LocalOptimizer
{
public:
    int num_agents;
    int window_size_for_PATH;
    std::mt19937 MT;
    std::shared_ptr<Instance> instance;
    std::shared_ptr<TimeSpaceAStarPlanner> planner;
    std::vector<int> goal_locs;  // Current goals for all agents

    LocalOptimizer() = default;
    
    LocalOptimizer(int num_agents, int window_size_for_PATH, int random_seed = 0)
        : num_agents(num_agents), window_size_for_PATH(window_size_for_PATH), MT(random_seed) {}

    // Initialize with instance
    void init(const std::shared_ptr<Instance>& inst, const std::vector<int>& goals) {
        instance = inst;
        goal_locs = goals;
        planner = std::make_shared<TimeSpaceAStarPlanner>(instance, window_size_for_PATH);
    }

    // Replan paths for a set of destroyed agents
    // agents_to_destroy: set of agent IDs to replan
    // paths: current paths for all agents (will be updated for destroyed agents)
    // path_table: conflict detection table
    // time_limit: time limit for planning
    // Returns: true if improved solution found
    bool optimize(
        const std::unordered_set<int>& agents_to_destroy,
        std::vector<std::vector<std::pair<int, int>>>& paths,
        PathTable& path_table,
        double time_limit
    ) {
        if (agents_to_destroy.empty() || !planner) return false;

        // Collect agent IDs to replan
        std::vector<int> agent_ids(agents_to_destroy.begin(), agents_to_destroy.end());
        
        // Collect starts and goals for agents to replan
        std::vector<int> starts;
        std::vector<int> goals;
        for (int agent_id : agent_ids) {
            if (agent_id < 0 || agent_id >= num_agents) continue;
            if (paths[agent_id].empty()) {
                starts.push_back(-1);  // Invalid
            } else {
                starts.push_back(paths[agent_id][0].first);
            }
            goals.push_back(goal_locs.size() > (size_t)agent_id ? goal_locs[agent_id] : -1);
        }

        // Replan paths
        std::vector<std::vector<std::pair<int, int>>> new_paths;
        planner->planPaths(agent_ids, starts, goals, path_table, new_paths);

        // Calculate old cost and new cost
        float old_cost = 0;
        float new_cost = 0;
        
        for (int agent_id : agent_ids) {
            if (agent_id < 0 || agent_id >= num_agents) continue;
            
            // Old cost (path length)
            if (!paths[agent_id].empty()) {
                old_cost += paths[agent_id].size();
            }
            
            // New cost
            size_t idx = &agent_id - &agent_ids[0];  // Map agent_id to index in new_paths
            if (idx < new_paths.size() && !new_paths[idx].empty()) {
                // Update path
                paths[agent_id] = new_paths[idx];
                new_cost += new_paths[idx].size();
            }
        }

        // Accept if improved (shorter path = better)
        return new_cost < old_cost;
    }

    // Reset optimizer state
    void reset() {
        // Reset any per-iteration state
    }

    // Update goal locations
    void updateGoals(const std::vector<int>& goals) {
        goal_locs = goals;
    }
};

} // namespace LNS
