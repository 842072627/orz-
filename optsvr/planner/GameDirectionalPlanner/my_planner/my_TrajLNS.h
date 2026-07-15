#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "my_Types.h"
#include "my_Memory.h"
#include "my_search_node.h"
#include "my_heap.h"
#include "my_heuristics.h"
#include <iostream>

#include <set>

namespace MyPlanner{
// enum ADAPTIVE {RANDOM, CONGESTION, COUNT};
enum ADAPTIVE {RANDOM, CONGESTION, DEVIATION, COUNT};

// extern std::vector<HeuristicTable> global_heuristictable;
extern std::unordered_map<int, HeuristicTable> global_heuristictable;
extern bool global_heuristics_initialized;
extern Neighbors global_neighbors;

struct FW_Metric{
    int id;
    int deviation;
    int last_replan_t;
    int rand;

    FW_Metric(int i, int d, int l) : id(i), deviation(d),last_replan_t(l){};
    FW_Metric(){};
};

struct FlowHeuristic{
    HeuristicTable* h; 
    int target;
    int origin;
    pqueue_min_of open;
    MemoryPool mem;


    bool empty(){
        return mem.generated() == 0;
    }
    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;

    TimePoint start_time;
    int t_ms=0;

    std::vector<Traj> trajs;

    std::vector<std::pair<int,int>> deviation_agents;

    std::vector<Int4> flow;
    // std::vector<HeuristicTable>& heuristics;
    std::unordered_map<int, HeuristicTable>& heuristics;
    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.

    std::vector<FW_Metric> fw_metrics;
    Neighbors& neighbors;

    std::vector<int> guide_failed_goal;
    std::vector<int> guide_failed_start;
    std::vector<int> guide_failed_until;
    std::vector<int> guide_failed_count;

    int traj_inited = 0;
    int dist2path_inited = 0;
    int soc = 0;

    MemoryPool mem;

    void init_mem(){
        mem.init(env->map.size());
    }

    bool guide_update_on_cooldown(int agent) const{
        if (env == nullptr || agent < 0 || agent >= (int)guide_failed_until.size() ||
            agent >= (int)tasks.size() || agent >= (int)env->curr_states.size()){
            return false;
        }
        const int start = env->curr_states[agent].location;
        return guide_failed_goal[agent] == tasks[agent] &&
               guide_failed_start[agent] == start &&
               env->curr_timestep < guide_failed_until[agent];
    }

    void mark_guide_update_failed(int agent, int cooldown_steps){
        if (env == nullptr || agent < 0 || agent >= (int)guide_failed_until.size() ||
            agent >= (int)tasks.size() || agent >= (int)env->curr_states.size()){
            return;
        }
        if (cooldown_steps < 1){
            cooldown_steps = 1;
        }
        guide_failed_goal[agent] = tasks[agent];
        guide_failed_start[agent] = env->curr_states[agent].location;
        guide_failed_until[agent] = env->curr_timestep + cooldown_steps;
        guide_failed_count[agent] += 1;
    }

    void clear_guide_update_failed(int agent){
        if (agent < 0 || agent >= (int)guide_failed_until.size()){
            return;
        }
        guide_failed_goal[agent] = -1;
        guide_failed_start[agent] = -1;
        guide_failed_until[agent] = -1;
        guide_failed_count[agent] = 0;
    }

    TrajLNS(SharedEnvironment* env, std::unordered_map<int, HeuristicTable>& heuristics, Neighbors& neighbors):
        env(env),
        trajs(env->num_of_agents),
        tasks(env->num_of_agents),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(heuristics),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),
        fw_metrics(env->num_of_agents),neighbors(neighbors),
        guide_failed_goal(env->num_of_agents, -1),
        guide_failed_start(env->num_of_agents, -1),
        guide_failed_until(env->num_of_agents, -1),
        guide_failed_count(env->num_of_agents, 0){
        };


    TrajLNS():heuristics(global_heuristictable), neighbors(global_neighbors){};

};
}
#endif
