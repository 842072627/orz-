#include "pibt.h"

#include <cstdint>
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "lane_data.h"
#include "lane_traffic_light.h"
#include "iron_highway.h"

// wait 原因追踪
int g_wait_no_successor = 0;
int g_wait_cycle = 0;



namespace OptPlan{
int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

	// 优先使用 guide path 的距离表（与 turn_aware 无关，这个表本身是准确的）
	if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
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
	  std::vector<bool>& occupied, TrajLNS& lns
	  ){
	// The PIBT works like a causal PIBT when using MAPF-T model. But a normal PIBT when using MAPF model.
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);
	const bool forward_break_tie = optsvr::cfg().pibt_forward_break_tie;

	if (higher_id == -1){
		//fist agent in the chain, disallow cyclic movments by preventing other agents moving to first agent's previous location
		occupied[prev_loc] = true;
	}

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));

		// Iron lane traffic light: block counterflow entry
		if (optsvr::cfg().iron_lane_enabled) {
			if (!IronLane::is_lane_cell(prev_loc) && IronLane::is_lane_cell(neighbor)
			    && !IronLane::g_exempt_from_traffic_light[curr_id]) {
				int lock = IronLane::g_lane_lock_dir;
				if (lock != 0) {
					int comp = IronLane::get_component(prev_loc);
					int dir = (comp == 2) ? -1 : (comp == 1) ? +1 : 0;
					if (dir != 0 && dir != lock) {
						if (dir > 0) IronLane::g_blocked_12++;
						else IronLane::g_blocked_21++;
						continue;  // counterflow → FORBID
					}
					if (dir > 0) IronLane::g_passed_12++;
					else if (dir < 0) IronLane::g_passed_21++;
				} else if (IronLane::g_clearance_mode) {
					// Clearance: block ALL entry, don't increment stats
					continue;
				}
			}
		}

		// Iron highway: check direction constraint on current cell
		{
			uint8_t forbid = IronHighway::get_forbid_mask(prev_loc);
			if (forbid != 0) {
				int delta = neighbor - prev_loc;
				// dir: 0=E, 1=S, 2=W, 3=N
				int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == lns.env->cols) ? 1 : 3;
				// forbid mask bits: 1=W, 2=E, 4=N, 8=S
				bool blocked = false;
				if (dir == 0 && (forbid & 2)) blocked = true;   // E blocked
				if (dir == 1 && (forbid & 8)) blocked = true;   // S blocked
				if (dir == 2 && (forbid & 1)) blocked = true;   // W blocked
				if (dir == 3 && (forbid & 4)) blocked = true;   // N blocked
				if (blocked) {
				    if (optsvr::IRON_HIGHWAY_PIBT_PENALTY > 0) {
				        int penalty_h = get_gp_h(lns, curr_id, neighbor) + optsvr::IRON_HIGHWAY_PIBT_PENALTY;
				        successors.emplace_back(neighbor, penalty_h, -1, rand());
				    }
				    continue;
				}
			}
		}

		int min_heuristic = get_gp_h(lns, curr_id, neighbor);

		successors.emplace_back(neighbor,min_heuristic,-1,rand());
	}

	int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);

	successors.emplace_back(prev_loc, wait_heuristic,-1,rand());

	// Debug: log PIBT state at target cells
	const int cols = lns.env->cols;
	const int cur_x = prev_loc % cols;
	const int cur_y = prev_loc / cols;
	if (optsvr::IRON_HIGHWAY_DEBUG &&
	    ((cur_x == 1622 && cur_y == 1019) || (cur_x == 1623 && cur_y == 1019))) {
	    int tick = lns.env->curr_timestep;
	    int goal = lns.tasks.at(curr_id);
	    int gx = goal % cols, gy = goal / cols;
	    std::fprintf(stderr, "[PIBT_DBG] tick=%d agent=%d cell=(%d,%d) goal=(%d,%d) higher=%d\n",
	        tick, curr_id, cur_x, cur_y, gx, gy, higher_id);
	    // Show each neighbor's guide heuristic
	    const char* dir_name[4] = {"E","S","W","N"};
	    int d4[4] = {1, cols, -1, -cols};
	    for (int di = 0; di < 4; di++) {
	        int nb = prev_loc + d4[di];
	        uint8_t fb = IronHighway::get_forbid_mask(prev_loc);
	        bool hw_blocked = false;
	        if (fb) {
	            int dir_bit = (di == 0) ? 2 : (di == 1) ? 8 : (di == 2) ? 1 : 4;
	            if (fb & dir_bit) hw_blocked = true;
	        }
	        int hval = -1;
	        for (auto& s : successors) {
	            if (s.location == nb) { hval = s.heuristic; break; }
	        }
	        std::fprintf(stderr, "  %s: loc=%d guide=%d hw_block=%d",
	            dir_name[di], nb, hval, (int)hw_blocked);
	        // Check occupancy
	        if (occupied[nb]) std::fprintf(stderr, " occ=Y");
	        if (decision[nb] != -1) std::fprintf(stderr, " dec=%d", decision[nb]);
	        if (higher_id != -1 && prev_decision[nb] == higher_id) std::fprintf(stderr, " prev_match");
	        std::fprintf(stderr, "\n");
	    }
	    // Wait heuristic
	    int wait_h = wait_heuristic;
	    std::fprintf(stderr, "  W: guide=%d occ=%d dec=%d\n",
	        wait_h, (int)occupied[prev_loc], decision[prev_loc]);
	}


	std::sort(successors.begin(), successors.end(), 
		[&](const PIBT_C& a, const PIBT_C& b)
		{
			if (a.heuristic == b.heuristic){
					//tie break on prefer moving forward
					if (forward_break_tie && a.location==orien_next_v && b.location!=orien_next_v)
						return true;
					if (forward_break_tie && a.location!=orien_next_v && b.location==orien_next_v)
						return false;
					// random tie break
					return a.tie_breaker < b.tie_breaker;
			}
			return a.heuristic < b.heuristic; 
		});
	
	// Highway force move: single-direction cell, target free → force move (skip occupied/decision checks)
	if (optsvr::IRON_HIGHWAY_ENABLED && optsvr::IRON_HIGHWAY_FORCE_MOVE) {
		uint8_t forbid = IronHighway::get_forbid_mask(prev_loc);
		if (forbid == 7 || forbid == 11 || forbid == 13 || forbid == 14) {
			for (auto& s : successors) {
				if (s.location == prev_loc) continue;  // skip wait
				int delta = s.location - prev_loc;
				int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == lns.env->cols) ? 1 : 3;
				// Check if successor is in the highway-allowed direction
				bool hw_allow = false;
				if (dir == 0 && !(forbid & 2)) hw_allow = true;
				if (dir == 1 && !(forbid & 8)) hw_allow = true;
				if (dir == 2 && !(forbid & 1)) hw_allow = true;
				if (dir == 3 && !(forbid & 4)) hw_allow = true;
				if (!hw_allow) continue;
				// Target free if: no occupant, or occupant already moved away
				int occupant = prev_decision[s.location];
				bool target_free = (occupant == -1 || occupant == curr_id);
				if (!target_free && next_states[occupant].location != -1 &&
				    next_states[occupant].location != s.location) {
					target_free = true;  // occupant moved elsewhere, target is empty
				}
				if (target_free) {
					next_states.at(curr_id) = State(s.location, -1, -1);
					decision.at(s.location) = curr_id;
					if (optsvr::IRON_HIGHWAY_DEBUG) {
						std::fprintf(stderr, "[HW_FORCE] tick=%d agent=%d from=%d to=%d\n",
							lns.env->curr_timestep, curr_id, prev_loc, s.location);
					}
					return true;
				}
				break;  // only check first (best-heuristic) highway-allowed successor
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
            if (!causalPIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns)){
				continue;
            }
        }

        return true;
    }

    next_states.at(curr_id) = State(prev_loc,-1 ,-1);;
    decision.at(prev_loc) = curr_id;
    g_wait_no_successor++;
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
	g_wait_cycle++;
	return false;
	

	

	

	
}

}
