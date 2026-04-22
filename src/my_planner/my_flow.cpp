#include "my_flow.h"
#include "my_sipp.h"

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

    // A112: Use map-specific neighbor_size (like Team_RAPID)
    // For large maps (>1000 agents): use all-agents approach (original)
    // For smaller maps: use neighbor-based approach (8 neighbors)
    const int agent_count = lns.env->num_of_agents;
    const int neighbor_size = 8;  // default for all maps
    
    auto now = std::chrono::steady_clock::now();
    auto remaining_ms = std::chrono::duration<double, std::milli>(timelimit - now).count();
    
    // Large maps need full coverage, use original all-agents approach
    if (agent_count > 1000) {
        int deadline_per_agent = std::max(5, (int)(remaining_ms / agent_count));
        int count = 0;
        int a, index;
        while (std::chrono::steady_clock::now() < timelimit){
            index = count % agent_count;
            a = replan_order[index].id;
            count++;
            if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
                continue;
            }
            std::vector<int> old_traj = lns.trajs[a];
            if (!lns.trajs[a].empty())
                remove_traj(lns, a);
            TimePoint agent_deadline = std::chrono::steady_clock::now() + 
                std::chrono::milliseconds(deadline_per_agent);
            if (!update_traj(lns, a, &agent_deadline)){
                lns.trajs[a] = old_traj;
                if (!old_traj.empty()){
                    add_traj(lns,a);
                }
                continue;
            }
        }
    } else {
        // Smaller maps (<1000 agents): use neighbor-based approach (like Team_RAPID)
        // A118 fix: Process top deviation agents with more time per agent
        int processed_count = 0;
        
        while (std::chrono::steady_clock::now() < timelimit && processed_count < agent_count) {
            // Give more time to each agent: remaining_ms / neighbor_size
            int deadline_per_agent = std::max(10, (int)(remaining_ms / neighbor_size));
            
            // Process up to neighbor_size agents starting from processed_count
            int start_idx = processed_count;
            int end_idx = std::min(start_idx + neighbor_size, agent_count);
            int processed_in_round = 0;
            
            for (int idx = start_idx; idx < end_idx && std::chrono::steady_clock::now() < timelimit; idx++) {
                int a = replan_order[idx].id;
                processed_in_round++;
                processed_count++;
                
                if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
                    continue;
                }
                std::vector<int> old_traj = lns.trajs[a];
                if (!lns.trajs[a].empty())
                    remove_traj(lns, a);
                TimePoint agent_deadline = std::chrono::steady_clock::now() + 
                    std::chrono::milliseconds(deadline_per_agent);
                if (!update_traj(lns, a, &agent_deadline)){
                    lns.trajs[a] = old_traj;
                    if (!old_traj.empty()){
                        add_traj(lns,a);
                    }
                    continue;
                }
            }
            
            // Break if we've processed all agents or no progress made
            if (processed_in_round == 0) break;
        }
    }
    return;

}




void update_dist_2_path(TrajLNS& lns, int i){
    init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
}

//compute distance table for each traj
void init_dist_table(TrajLNS& lns){
    // Team_Kitty_Knight: process all agents (no amount limit)
    for (int i=0 ; i <lns.env->num_of_agents; i++){
        if(!lns.trajs[i].empty() && lns.trajs[i].size() == get_heuristic(lns.heuristics[lns.trajs[i].back()], lns.env,lns.trajs[i].front(),&(lns.neighbors)))
            continue;
        if(!lns.trajs[i].empty() && lns.traj_dists[i].empty()){
            init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
            lns.dist2path_inited++;
        }
    }
}

//update traj and distance table for agent i (backward compatible 3-arg version)
bool update_traj_3args(TrajLNS& lns, int i, const TimePoint* deadline){
    return update_traj(lns, i, deadline, false);
}

//update traj and distance table for agent i
// use_sipp parameter controls whether to use SIPP (temporal conflict avoidance)
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline, bool use_sipp){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];
    
    if (use_sipp) {
        // A138: Use SIPP for maps that need temporal conflict handling
        std::vector<int> path;
        if (findPathSIPP(lns.env, start, goal, path, 500)) {
            lns.trajs[i] = path;
            return true;
        }
        return false;
    }
    
    s_node goal_node = astar(lns.env,lns.flow, lns.heuristics[goal],lns.trajs[i],lns.mem,start,goal, &(lns.neighbors), deadline);
    if (goal_node.id == -1){
        return false;
    }
    lns.goal_nodes[i] = goal_node;
    add_traj(lns,i);
    update_dist_2_path(lns,i);
    return true;
}

}
