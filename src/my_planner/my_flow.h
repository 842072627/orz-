
#ifndef flow_hpp
#define flow_hpp
#include "my_Types.h"
#include "my_search.h"
#include "my_TrajLNS.h"
#include "my_heuristics.h"

#include <random>
#include <unordered_set>

namespace MyPlanner{

//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent);

void add_traj(TrajLNS& lns, int agent);


void get_deviation(TrajLNS& lns);

void update_fw_metrics(TrajLNS& lns);

void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit);

void update_dist_2_path(TrajLNS& lns, int i);

//compute distance table for each traj
void init_dist_table(TrajLNS& lns);

//update traj and distance table for agent i (backward compatible 3-arg version)
bool update_traj_3args(TrajLNS& lns, int i, const TimePoint* deadline);

//update traj and distance table for agent i (with SIPP option)
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline, bool use_sipp);

//inline wrapper for backward compatibility
inline bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline) {
    return update_traj_3args(lns, i, deadline);
}


}
#endif
