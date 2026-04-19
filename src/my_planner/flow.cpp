

#include "flow.h"


#include <random>
#include <unordered_set>

namespace MyPlanner{

std::mt19937 g(0);


//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent){
    lns.soc -= lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d, to;

    to = lns.trajs[agent].size();

    for (int j = 1; j < to; j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);


        lns.flow[prev_loc].d[d] -= 1;

    }
}

void add_traj(TrajLNS& lns, int agent){

    //update last replan time for agent
    lns.fw_metrics[agent].last_replan_t = lns.env->curr_timestep;

    lns.soc += lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d;
    for (int j = 1; j < lns.trajs[agent].size(); j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] += 1;

    }
}


void get_deviation(TrajLNS& lns){
    lns.deviation_agents.clear();
    for  (int i=0; i< lns.env->num_of_agents;i++){
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;

        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));

        if (dists.first > 0){
            lns.deviation_agents.emplace_back(dists.first, i);
        }
    }

    std::sort(lns.deviation_agents.begin(), lns.deviation_agents.end(),
        [](std::pair<int,int>& a, std::pair<int,int>& b)
		{
            return a.first > b.first;
        });
    return;
}

void update_fw_metrics(TrajLNS& lns){
    for  (int i=0; i< lns.env->num_of_agents;i++){
        lns.fw_metrics[i].id = i;
        lns.fw_metrics[i].rand = rand();
        lns.fw_metrics[i].deviation = 0;
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;
        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));
        assert(dists.first >= 0 );
        lns.fw_metrics[i].deviation = dists.first;
    }
    return;
}


void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit){
    update_fw_metrics(lns);
    std::vector<FW_Metric> replan_order = lns.fw_metrics;
    assert(replan_order.size() == lns.env->num_of_agents);

    std::sort(replan_order.begin(), replan_order.end(),
    [](FW_Metric& a, FW_Metric& b)
    {
        if (a.deviation > b.deviation)
            return true;
        else if ( a.deviation < b.deviation)
            return false;
        
        if (a.last_replan_t < b.last_replan_t)
            return true;
        else if (a.last_replan_t > b.last_replan_t)
            return false;

        return a.rand > b.rand;
    });

    int count=0;
    int a, index;
    while (std::chrono::steady_clock::now() < timelimit){
        index = count%lns.env->num_of_agents;
        a = replan_order[index].id;
        count++;
        if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
            continue;
        }
        std::vector<int> old_traj = lns.trajs[a];
        remove_traj(lns,a);
        if (!update_traj(lns,a, &timelimit)){
            lns.trajs[a] = old_traj;
            add_traj(lns,a);
            break;
        }
        
    }
    return;

}




void update_dist_2_path(TrajLNS& lns, int i){
    init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
}

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount){

    int count = 0;
    for (int i=0 ; i <lns.env->num_of_agents; i++){
                // std::cout<<i<<";";
        if (count >= amount){
            break;
        }
        if(!lns.trajs[i].empty() && lns.trajs[i].size() == get_heuristic(lns.heuristics[lns.trajs[i].back()], lns.env,lns.trajs[i].front(),&(lns.neighbors)))
            continue;
        if(!lns.trajs[i].empty() && lns.traj_dists[i].empty()){
            init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
            count++;
            lns.dist2path_inited++;
        }

    }
}

//update traj and distance table for agent i
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];
    s_node goal_node = astar(lns.env,lns.flow, lns.heuristics[goal],lns.trajs[i],lns.mem,start,goal, &(lns.neighbors), deadline);
    if (goal_node.id == -1){
        return false;
    }
    lns.goal_nodes[i] = goal_node;
    add_traj(lns,i);
    update_dist_2_path(lns,i);
    return true;
}

// Multi-iteration LNS-style neighbor replanning
// Iteratively find agents with high deviation and replan their paths
void lns_replan_neighbors(TrajLNS& lns, int num_neighbors, const TimePoint* deadline){
    const int max_iterations = 2;
    
    for (int iter = 0; iter < max_iterations; iter++){
        // Check time limit
        if (deadline && std::chrono::steady_clock::now() >= *deadline){
            break;
        }
        
        // Calculate deviations for all agents
        std::vector<std::pair<int, int>> deviations; // (agent_id, deviation)
        
        for (int i = 0; i < lns.env->num_of_agents; i++){
            int start = lns.env->curr_states[i].location;
            int goal = lns.tasks[i];
            
            if (lns.trajs[i].empty()) continue;
            
            // Get expected distance (manhattan or heuristic)
            int expected_dist = get_heuristic(lns.heuristics[goal], lns.env, start, &(lns.neighbors));
            
            // Get actual path length
            int actual_dist = lns.trajs[i].size();
            
            // Deviation = actual - expected (higher = more delayed)
            int deviation = actual_dist - expected_dist;
            if (deviation > 0){
                deviations.push_back({i, deviation});
            }
        }
        
        // Sort by deviation descending
        std::sort(deviations.begin(), deviations.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });
        
        // Replan top N neighbors
        int count = 0;
        for (const auto& dev : deviations){
            if (count >= num_neighbors) break;
            
            int i = dev.first;
            
            // Save old trajectory
            auto old_traj = lns.trajs[i];
            
            // Remove old trajectory from flow
            remove_traj(lns, i);
            
            // Try to replan
            if (update_traj(lns, i, deadline)){
                // Check if new path is shorter
                if (lns.trajs[i].size() < old_traj.size()){
                    // Improvement found, keep new path
                    count++;
                } else {
                    // No improvement, restore old path
                    lns.trajs[i] = old_traj;
                    add_traj(lns, i);
                }
            } else {
                // Replan failed, restore old path
                lns.trajs[i] = old_traj;
                add_traj(lns, i);
            }
        }
        
        // Early exit if no improvements made
        if (count == 0) break;
    }
}

}
