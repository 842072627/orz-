#include "pibt.h"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include "RuntimeConfig.h"
#include "Config.h"
#include "HeuristicMatrix.h"
#include "TestUsefulPibtDensity.h"
#include "fullfill_highway.h"
#include "heuristics.h"




namespace TestUsefulFlowDecayPlan{
int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

	// 优先使用 guide path 的距离表（与 turn_aware 无关，这个表本身是准确的）
	if (ai >= 0 &&
		ai < static_cast<int>(lns.traj_dists.size()) && !lns.traj_dists[ai].empty())
	{
		return get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors));
	}

	// turn_aware 模式：用预计算矩阵（不传朝向 → 取 4 方向最小值）
	if (optsvr::cfg().turn_aware_heuristic) {
		return optsvr::get_h_fast(lns.env, target, lns.tasks.at(ai));
	}

	// 原始逻辑
	if (is_large_map(lns.env)){
		min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
		return min_heuristic;
	}
	else{
		if (!lns.heuristics[lns.tasks.at(ai)].empty())
		{
			min_heuristic = get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));
		}
		else
		{
			min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
		}
		return min_heuristic;
	}
}

bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	  std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, TrajLNS& lns,
	  const optsvr::TestUsefulPibtDensity* history_congestion
	  ){
	// The PIBT works like a causal PIBT when using MAPF-T model. But a normal PIBT when using MAPF model.
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);

	if (higher_id == -1){
		//fist agent in the chain, disallow cyclic movments by preventing other agents moving to first agent's previous location
		occupied[prev_loc] = true;
	}

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<PIBT_C> successors;
	auto add_successor = [&](int loc, bool is_wait) {
		double base_heuristic = static_cast<double>(get_gp_h(lns, curr_id, loc));
		double history_penalty = 0.0;
		if (history_congestion != nullptr) {
			history_penalty = history_congestion->candidate_penalty(loc, base_heuristic, is_wait);
		}
		successors.emplace_back(
			loc,
			base_heuristic + history_penalty,
			-1,
			rand(),
			base_heuristic,
			history_penalty,
			is_wait);
	};

	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));
		// Fullfill highway: hard forbid direction check
		{
		    uint8_t forbid = TestUsefulFlowDecayPlan::get_highway_forbid(prev_loc);
		    if (forbid != 0) {
		        int delta = neighbor - prev_loc;
		        int cols = lns.env->cols;
		        int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == cols) ? 1 : 3;
		        bool blocked = false;
		        if (dir == 0 && (forbid & 2)) blocked = true;
		        if (dir == 1 && (forbid & 8)) blocked = true;
		        if (dir == 2 && (forbid & 1)) blocked = true;
		        if (dir == 3 && (forbid & 4)) blocked = true;
		        if (blocked) continue;
		    }
		}
		add_successor(neighbor, false);
	}

	add_successor(prev_loc, true);

	const int history_sort_mode = history_congestion != nullptr ? history_congestion->sort_mode() : 0;
	const double history_tie_eps = history_congestion != nullptr ? history_congestion->tie_base_eps() : 0.0;
	if (history_congestion != nullptr && history_sort_mode == 2 && !successors.empty()) {
		double best_base = std::numeric_limits<double>::infinity();
		for (const auto& candidate: successors) {
			best_base = std::min(best_base, candidate.base_heuristic);
		}
		for (auto& candidate: successors) {
			if (candidate.base_heuristic <= best_base + history_tie_eps + 1e-9) {
				candidate.heuristic = candidate.base_heuristic + candidate.history_penalty;
			} else {
				candidate.heuristic = candidate.base_heuristic;
			}
		}
	}

	auto prefer_forward_or_random = [&](const PIBT_C& a, const PIBT_C& b) {
		if (a.location==orien_next_v && b.location!=orien_next_v)
			return true;
		if (a.location!=orien_next_v && b.location==orien_next_v)
			return false;
		return a.tie_breaker < b.tie_breaker;
	};
	auto base_less = [&](const PIBT_C& a, const PIBT_C& b) {
		if (std::fabs(a.base_heuristic - b.base_heuristic) < 1e-9) {
			return prefer_forward_or_random(a, b);
		}
		return a.base_heuristic < b.base_heuristic;
	};

	int base_first_loc = -1;
	bool base_first_wait = false;
	if (history_congestion != nullptr && !successors.empty()) {
		const auto base_first_it = std::min_element(successors.begin(), successors.end(), base_less);
		base_first_loc = base_first_it->location;
		base_first_wait = base_first_it->is_wait;
	}

	std::sort(successors.begin(), successors.end(),
		[&](const PIBT_C& a, const PIBT_C& b)
		{
			if (history_congestion != nullptr && history_sort_mode == 1) {
				if (std::fabs(a.base_heuristic - b.base_heuristic) > history_tie_eps + 1e-9) {
					return a.base_heuristic < b.base_heuristic;
				}
				if (std::fabs(a.history_penalty - b.history_penalty) > 1e-9) {
					return a.history_penalty < b.history_penalty;
				}
				return prefer_forward_or_random(a, b);
			}
			if (std::fabs(a.heuristic - b.heuristic) < 1e-9){
				return prefer_forward_or_random(a, b);
			}
			return a.heuristic < b.heuristic;
		});

	if (history_congestion != nullptr && !successors.empty()) {
		history_congestion->record_ordering(
			base_first_loc != successors.front().location,
			base_first_wait,
			successors.front().is_wait);
	}

	// Fullfill highway force move (config FULLFILL_HIGHWAY_FORCE_MOVE, default off)
	// Warehouse has too many single-direction cells — force move causes cascading deadlocks.
	if (optsvr::FULLFILL_HIGHWAY_FORCE_MOVE) {
	    uint8_t forbid = TestUsefulFlowDecayPlan::get_highway_forbid(prev_loc);
	    if (forbid == 7 || forbid == 11 || forbid == 13 || forbid == 14) {
	        for (auto& s : successors) {
	            if (s.location == prev_loc) continue;
	            int delta = s.location - prev_loc;
	            int cols = lns.env->cols;
	            int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == cols) ? 1 : 3;
	            bool hw_allow = false;
	            if (dir == 0 && !(forbid & 2)) hw_allow = true;
	            if (dir == 1 && !(forbid & 8)) hw_allow = true;
	            if (dir == 2 && !(forbid & 1)) hw_allow = true;
	            if (dir == 3 && !(forbid & 4)) hw_allow = true;
	            if (!hw_allow) continue;
	            int occupant = prev_decision[s.location];
	            bool target_free = (occupant == -1 || occupant == curr_id);
	            if (!target_free && next_states[occupant].location != -1 &&
	                next_states[occupant].location != s.location) {
	                target_free = true;
	            }
	            if (target_free) {
	                next_states.at(curr_id) = State(s.location, -1, -1);
	                decision.at(s.location) = curr_id;
	                return true;
	            }
	        }
	    }
	}

    for (auto& next: successors){
		if(occupied[next.location] && !(higher_id == -1 && prev_loc == next.location))
			continue;
		assert(validateMove(prev_loc, next.location, lns.env));
		
		if (next.location == -1)
			continue;
		if (decision[next.location] != -1){
			continue;
		}
		if (higher_id != -1 && prev_decision[next.location] == higher_id){
			continue;
		}
		next_states.at(curr_id) = State(next.location, -1, -1);
		decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 && 
			next_states.at(prev_decision.at(next.location)).location == -1){
            int lower_id = prev_decision.at(next.location);
            if (!causalPIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns,history_congestion)){
				continue;
            }
        }

        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);;
    decision.at(prev_loc) = curr_id;     
    return false;
}

Action getAction(State& prev, State& next){
	if (prev.location == next.location && prev.orientation == next.orientation){
		return Action::W;
	}
	if (prev.location != next.location && prev.orientation == next.orientation){
		return Action::FW;
	}
	if (next.orientation  == (prev.orientation+1)%4){
		return Action::CR;
	}
	if (next.orientation  == (prev.orientation+3)%4){
		return Action::CCR;
	}
	assert(false);
	return Action::W;
}

Action getAction(State& prev, int next_loc, SharedEnvironment* env){
	if (prev.location == next_loc){
		return Action::W;
	}
	int diff = next_loc -prev.location;
	int orientation;
	if (diff == 1){
		orientation = 0;
	}
	if (diff == -1){
		orientation = 2;
	}
	if (diff == env->cols){
		orientation = 1;
	}
	if (diff == -env->cols){
		orientation = 3;
	}
	if (orientation == prev.orientation){
		return Action::FW;
	}
	if (orientation  == (prev.orientation+1)%4){
		return Action::CR;
	}
	if (orientation  == (prev.orientation+3)%4){
		return Action::CCR;
	}
	if (orientation  == (prev.orientation+2)%4){
		return Action::CR;
	}
	assert(false);
	return Action::W;

}

bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision){
	if (checked.at(id) && actions.at(id) == Action::FW)
		return true;
	checked.at(id) = true;

	if (actions.at(id) != Action::FW)
		return false;

	//move forward
	int target = decided.at(id).loc;
	assert(target != -1);

	int na = prev_decision[target];
	if (na == -1)
		return true;

	if (moveCheck(na,checked,decided,actions,prev_decision))
		return true;
	actions.at(id) = Action::W;
	return false;
	

	

	
}
}
