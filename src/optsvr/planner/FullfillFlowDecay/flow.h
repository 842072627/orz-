
#ifndef TESTUSEFUL_FLOWDECAY_flow_hpp
#define TESTUSEFUL_FLOWDECAY_flow_hpp
#include "Types.h"
#include "search.h"
#include "TrajLNS.h"
#include "heuristics.h"

#include <functional>
#include <random>
#include <unordered_set>

namespace FullfillFlowDecayPlan{

//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent);

void add_traj(TrajLNS& lns, int agent);


void get_deviation(TrajLNS& lns);

void update_fw_metrics(TrajLNS& lns);

void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit,
    const std::vector<std::vector<int>>* extra_weights = nullptr,
    const std::function<void(int)>& on_update = nullptr);

void update_dist_2_path(TrajLNS& lns, int i);

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount);

//update traj and distance table for agent i
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline = nullptr,
    const std::vector<std::vector<int>>* extra_weights = nullptr);


}
#endif
