#include "LocalOptimizer.h"
#include <algorithm>

namespace LNS {

LocalOptimizer::LocalOptimizer(
    Instance& instance,
    TrajLNS& lns,
    std::string replan_algo_name,
    int window_size_for_PATH
) : instance(instance),
    lns(lns),
    replan_algo_name(replan_algo_name),
    window_size_for_PATH(window_size_for_PATH)
{
}

void LocalOptimizer::reset() {
    // Reset state if needed
}

bool LocalOptimizer::runPP(Neighbor& neighbor) {
    // Prioritized Planning (PP) replanning
    // For each agent in the neighbor set, replan its path
    
    if (replan_algo_name != "PP") {
        // Fall back to simple greedy
        return runPP(neighbor);
    }
    
    // Store old paths
    neighbor.m_old_paths.resize(neighbor.agents.size());
    neighbor.old_sum_of_costs = 0;
    
    for (size_t i = 0; i < neighbor.agents.size(); i++) {
        int agent_id = neighbor.agents[i];
        if (agent_id < 0 || agent_id >= instance.num_of_agents) continue;
        
        neighbor.m_old_paths[i] = lns.trajs[agent_id];
        neighbor.old_sum_of_costs += lns.trajs[agent_id].size();
    }
    
    // Simple replanning: use shortest path for each agent
    // This is a simplified version - real PP would use prioritized planning
    neighbor.m_paths.resize(neighbor.agents.size());
    neighbor.sum_of_costs = 0;
    
    for (size_t i = 0; i < neighbor.agents.size(); i++) {
        int agent_id = neighbor.agents[i];
        if (agent_id < 0 || agent_id >= instance.num_of_agents) continue;
        
        Traj new_path;
        int start_loc = instance.start_locations[agent_id];
        int goal_loc = instance.goal_locations[agent_id];
        
        // Simple BFS shortest path
        // In a real implementation, this would use A* or PIBT
        std::vector<int> dist(instance.map_size, -1);
        std::vector<int> parent(instance.map_size, -1);
        std::vector<int> queue;
        
        dist[start_loc] = 0;
        queue.push_back(start_loc);
        
        while (!queue.empty()) {
            int curr = queue.front();
            queue.erase(queue.begin());
            
            if (curr == goal_loc) break;
            
            auto nbrs = instance.getNeighbors(curr);
            for (int nxt : nbrs) {
                if (nxt < 0 || nxt >= instance.map_size) continue;
                if (instance.isObstacle(nxt)) continue;
                if (dist[nxt] >= 0) continue;
                
                dist[nxt] = dist[curr] + 1;
                parent[nxt] = curr;
                queue.push_back(nxt);
            }
        }
        
        // Reconstruct path
        if (dist[goal_loc] >= 0) {
            int curr = goal_loc;
            while (curr != start_loc) {
                new_path.push_back(curr);
                curr = parent[curr];
                if (curr < 0) break;
            }
            new_path.push_back(start_loc);
            std::reverse(new_path.begin(), new_path.end());
        }
        
        neighbor.m_paths[i] = new_path;
        neighbor.sum_of_costs += new_path.size();
    }
    
    neighbor.succ = (neighbor.sum_of_costs < neighbor.old_sum_of_costs);
    return neighbor.succ;
}

void LocalOptimizer::optimize(Neighbor& neighbor) {
    // Main optimization routine
    // Prepare and run replanning
    runPP(neighbor);
}

}
