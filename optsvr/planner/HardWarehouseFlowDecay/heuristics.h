
#ifndef HARD_WAREHOUSE_FLOWDECAY_heuristics_hpp
#define HARD_WAREHOUSE_FLOWDECAY_heuristics_hpp

#include "Types.h"
#include "utils.h"
#include <queue>
#include "TrajLNS.h"
#include "search_node.h"

namespace HardWarehouseFlowDecayPlan{

void init_heuristics(SharedEnvironment* env);

void init_neighbor(SharedEnvironment* env);

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location);

int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns);

int get_h(SharedEnvironment* env, int source, int target);


void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path);

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns);

// Highway forbid mask (fullfill_highway.h based)
// Store pre-computed forbid mask array for fast BFS checking
// forbid bits: 1=W, 2=E, 4=N, 8=S, 15=all blocked
void init_highway_forbid_mask(int map_size);
void init_highway_forbid_mask_from_weights(const std::vector<std::vector<int>>& extra_weights, int map_size);
uint8_t get_highway_forbid(int loc);

}
#endif