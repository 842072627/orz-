

#include "flow.h"

#include "RuntimeConfig.h"
#include <functional>
#include <random>
#include <unordered_set>
#include <cmath>

namespace OptPlan{

std::mt19937 g(0);

// 计算路径第 j 步（j 从 1 开始）的 flow 贡献值
// decay=1.0 && scale=1 时返回 1，与原始行为一致
static inline int flow_weight(int j, float decay, int scale) {
    if (decay >= 1.0f && scale <= 1) return 1;  // 快速路径：无衰减
    float w = static_cast<float>(scale) * std::pow(decay, static_cast<float>(j - 1));
    int iw = static_cast<int>(w + 0.5f);  // 四舍五入
    return (iw < 1) ? 1 : iw;
}

//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent){
    lns.soc -= lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d, to;
    float decay = optsvr::cfg().flow_decay;
    int   scale = optsvr::cfg().flow_decay_scale;

    to = lns.trajs[agent].size();

    for (int j = 1; j < to; j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] -= flow_weight(j, decay, scale);
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
    float decay = optsvr::cfg().flow_decay;
    int   scale = optsvr::cfg().flow_decay_scale;

    for (int j = 1; j < (int)lns.trajs[agent].size(); j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] += flow_weight(j, decay, scale);
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


int frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit,
    const std::vector<std::vector<int>>* extra_weights,
    int* total_successful_updates,
    const std::function<void(int)>& on_update){
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
    int successful_updates=0;
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
        if (!update_traj(lns,a, &timelimit, extra_weights)){
            lns.trajs[a] = old_traj;
            add_traj(lns,a);
            break;
        }
        updated.insert(a);
        successful_updates++;
        if (on_update) {
            on_update(a);
        }
        
    }
    if (total_successful_updates != nullptr) {
        *total_successful_updates = successful_updates;
    }
    return static_cast<int>(updated.size());

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
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline,
    const std::vector<std::vector<int>>* extra_weights){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];

    s_node goal_node = astar(lns.env, lns.flow, lns.heuristics[goal],
                             lns.trajs[i], lns.mem, start, goal,
                             &(lns.neighbors), deadline, extra_weights,
                             lns.env->curr_states[i].orientation);
    if (goal_node.id == -1){
        return false;
    }
    lns.goal_nodes[i] = goal_node;

    add_traj(lns,i);
    update_dist_2_path(lns,i);
    return true;
}

}
