

#include "heuristics.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "fullfill_highway.h"
#include <queue>


namespace TestUsefulFlowDecayPlan{

// std::vector<HeuristicTable> global_heuristictable; //TODO: Change this to unordered_map
std::unordered_map<int, HeuristicTable> global_heuristictable;
bool global_heuristics_initialized = false;
Neighbors global_neighbors;

// Highway forbid mask (pre-computed from extra_weights)
static std::vector<uint8_t> g_highway_forbid_mask;

void init_highway_forbid_mask(int map_size) {
    // Deprecated: kept for compatibility. Use init_highway_forbid_mask_from_weights instead.
    g_highway_forbid_mask.assign(map_size, 0);
}

void init_highway_forbid_mask_from_weights(const std::vector<std::vector<int>>& extra_weights, int map_size) {
    g_highway_forbid_mask.assign(map_size, 0);
    if (extra_weights.size() < static_cast<size_t>(map_size)) return;
    static constexpr int FORBID_THRESHOLD = 50000;  // anything above this is a hard forbid
    for (int loc = 0; loc < map_size; loc++) {
        uint8_t mask = 0;
        if (extra_weights[loc][0] >= FORBID_THRESHOLD) mask |= 2;   // E forbid
        if (extra_weights[loc][1] >= FORBID_THRESHOLD) mask |= 8;   // S forbid
        if (extra_weights[loc][2] >= FORBID_THRESHOLD) mask |= 1;   // W forbid
        if (extra_weights[loc][3] >= FORBID_THRESHOLD) mask |= 4;   // N forbid
        g_highway_forbid_mask[loc] = mask;
    }
}

uint8_t get_highway_forbid(int loc) {
    if (loc < 0 || loc >= static_cast<int>(g_highway_forbid_mask.size()))
        return 0;
    return g_highway_forbid_mask[loc];
}

void init_neighbor(SharedEnvironment* env){
	global_neighbors.resize(env->rows * env->cols);
	for (int row=0; row<env->rows; row++){
		for (int col=0; col<env->cols; col++){
			int loc = row*env->cols+col;
			if (env->map[loc]==0){
				if (row>0 && env->map[loc-env->cols]==0){
					global_neighbors[loc].push_back(loc-env->cols);
				}
				if (row<env->rows-1 && env->map[loc+env->cols]==0){
					global_neighbors[loc].push_back(loc+env->cols);
				}
				if (col>0 && env->map[loc-1]==0){
					global_neighbors[loc].push_back(loc-1);
				}
				if (col<env->cols-1 && env->map[loc+1]==0){
					global_neighbors[loc].push_back(loc+1);
				}
			}
		}
	}
};

void init_heuristics(SharedEnvironment* env){
	if (!global_heuristics_initialized){
		// global_heuristictable.resize(env->map.size());
		global_heuristics_initialized=true;
		init_neighbor(env);
	}
}

void init_heuristic(HeuristicTable& ht, SharedEnvironment* env, int goal_location){
	// initialize my_heuristic, but have error on malloc: Region cookie corrupted for region
	ht.htable.clear();
	ht.open.clear();
	// generate a open that can save nodes (and a open_handle)
	HNode root(goal_location,0, 0);
	ht.htable.insert_or_assign(goal_location, 0);
	ht.open.push_back(root);  // add root to open
}


int get_heuristic(HeuristicTable& ht, SharedEnvironment* env, int source, Neighbors* ns){
		auto it = ht.htable.find(source);
		if (it != ht.htable.end() && it->second < MAX_TIMESTEP) return it->second;

		std::vector<int> neighbors;
		int cost, diff;
		while (!ht.open.empty())
		{
			HNode curr = ht.open.front();
			ht.open.pop_front();

			
			getNeighborLocs(ns,neighbors,curr.location);

			
			for (int next : neighbors)
			{
				cost = curr.value + 1;
				diff = curr.location - next;
				
				assert(next >= 0 && next < env->map.size());
				//set current cost for reversed direction

				auto it = ht.htable.find(next);
				if (it != ht.htable.end() && it->second <= cost) // if 'next' is not found in htable, that means it's MAX_TIMESTEP
					continue;

				ht.open.emplace_back(next,0, cost);
				ht.htable.insert_or_assign(next, cost);
				
			}

			if (source == curr.location)
				return curr.value;
		}


		return MAX_TIMESTEP;
}

int get_h(SharedEnvironment* env, int source, int target){
	// 优先走 location-only 预计算矩阵
	auto& hm = optsvr::heuristic_matrix();
	if (hm.is_ready() && !hm.is_turn_aware()) {
		uint16_t d = hm.get(source, target);
		if (d != optsvr::DIST_UNREACHABLE) return static_cast<int>(d);
	}
	// turn_aware 矩阵或矩阵未就绪：回退到原始逻辑
	if (is_large_map(env))
		return manhattanDistance(source, target, env);
	else{
		// if (global_heuristictable.empty()){
		if (!global_heuristics_initialized){
			init_heuristics(env);
		}

		// if (global_heuristictable.at(target).empty()){
		if (global_heuristictable.find(target) == global_heuristictable.end() || global_heuristictable[target].empty()){
			global_heuristictable.insert_or_assign(target, HeuristicTable());
			init_heuristic(global_heuristictable[target],env,target);
		}

		return get_heuristic(global_heuristictable[target], env, source, &global_neighbors);
	}
}



void init_dist_2_path(Dist2Path& dp, SharedEnvironment* env, Traj& path){
	dp.open.clear();
	dp.label++;

    int togo = 0;
    for(int i = path.size()-1; i>=0; i--){
		int loc = path[i];

        // If this location has already been visited in this label, skip it
        auto it = dp.dist2path.find(loc);
        if (it != dp.dist2path.end() && it->second.label == dp.label) {
            assert(it->second.cost == MAX_TIMESTEP);
        }

        d2p entry(dp.label, loc, 0, togo);
        dp.open.emplace_back(entry);
        dp.dist2path.insert_or_assign(loc, entry);

        togo++;
    }

}

std::pair<int,int> get_source_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{
	auto it_source = dp.dist2path.find(source);
    if (it_source != dp.dist2path.end() && it_source->second.label == dp.label && it_source->second.cost < MAX_TIMESTEP) 
        return std::make_pair(it_source->second.cost, it_source->second.togo);
	
	std::vector<int> neighbors;
	int cost;

	while (!dp.open.empty())
	{
		d2p curr = dp.open.front();
		dp.open.pop_front();
		getNeighborLocs(ns,neighbors,curr.id);
		for (int next_location : neighbors)
		{
			// Highway forbid check in BFS spread
			uint8_t forbid = get_highway_forbid(curr.id);
			if (forbid != 0) {
			    int delta = next_location - curr.id;
			    int cols = env->cols;
			    int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == cols) ? 1 : 3;
			    bool blocked = false;
			    if (dir == 0 && (forbid & 2)) blocked = true;
			    if (dir == 1 && (forbid & 8)) blocked = true;
			    if (dir == 2 && (forbid & 1)) blocked = true;
			    if (dir == 3 && (forbid & 4)) blocked = true;
			    if (blocked) continue;
			}
			cost = curr.cost + 1;
			auto it_next = dp.dist2path.find(next_location);
            if (it_next != dp.dist2path.end() && it_next->second.label == dp.label && cost >= it_next->second.cost)
                continue;
			dp.open.emplace_back(dp.label,next_location,cost,curr.togo);
			dp.dist2path.insert_or_assign(next_location, d2p(dp.label,next_location,cost,curr.togo));
		}
		if (source == curr.id){
			return std::make_pair(curr.cost, curr.togo);
		}
	}
	return std::make_pair(MAX_TIMESTEP,0);
}

int get_dist_2_path(Dist2Path& dp, SharedEnvironment* env, int source, Neighbors* ns)
{

	std::pair<int, int> dists = get_source_2_path(dp,env, source, ns);

	return dists.first + dists.second;
}


}
