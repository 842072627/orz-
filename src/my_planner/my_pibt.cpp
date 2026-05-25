


#include "my_pibt.h"
#include "../pibt_lock.h"
#include "../centerline.h"
#include <cstdint>

namespace MyPlanner{

#if USE_WALL_HUGGING && USE_WALL_HUGGING_PIBT
// Scan from loc in direction (dx, dy) until hitting wall or map boundary
static int scan_to_wall(int loc, int dx, int dy, const SharedEnvironment* env) {
    int steps = 0;
    while (true) {
        int r = loc / env->cols + dy;
        int c = loc % env->cols + dx;
        int nxt = r * env->cols + c;
        if (r < 0 || r >= env->rows || c < 0 || c >= env->cols || env->map[nxt] != 0)
            return steps;
        loc = nxt;
        steps++;
        if (steps > 50) return steps;
    }
}
// Find main point si closest to given main_dist
static int find_mp_si(int a_m) {
    if (!CenterlineAccess::g_mp_main_dist || CenterlineAccess::g_mp_count <= 0)
        return -1;
    int best_si = 0, best_d = 999999;
    for (int si = 0; si < CenterlineAccess::g_mp_count; si++) {
        int d = std::abs((*CenterlineAccess::g_mp_main_dist)[si] - a_m);
        if (d < best_d) { best_d = d; best_si = si; }
    }
    return best_si;
}
#endif

int get_gp_h(TrajLNS& lns, int ai, int target){
    int min_heuristic;

	if (is_large_map(lns.env)){
		if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
		{
			min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors)); 
		}
		else
		{
			min_heuristic = manhattanDistance(target,lns.tasks.at(ai),lns.env);
		}
		return min_heuristic;
	}
	else{
		if (!lns.traj_dists.empty() && !lns.traj_dists[ai].empty())
		{
			min_heuristic = get_dist_2_path(lns.traj_dists[ai], lns.env, target, &(lns.neighbors)); 
		}
		else if (!lns.heuristics[lns.tasks.at(ai)].empty())
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

bool causalMY_PIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	  std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied, TrajLNS& lns
	  ){
	// The MY_PIBT works like a causal MY_PIBT when using MAPF-T model. But a normal MY_PIBT when using MAPF model.
	
	assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
	int prev_orientation = prev_states[curr_id].orientation;
	int next[4] = { prev_loc + 1,prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
	int orien_next_v = next[prev_orientation];

	assert(prev_loc >= 0 && prev_loc < lns.env->map.size());

	int target = lns.tasks.at(curr_id);

#if ORZ_PIBT_DIRECTION_BLOCK
	// If scheduler flagged this agent as direction-blocked, force wait
	if (curr_id < PIBT_Lock::MAX_AGENTS && PIBT_Lock::g_agent_blocked[curr_id]) {
		next_states.at(curr_id) = State(prev_loc, -1, -1);
		decision.at(prev_loc) = curr_id;
		return false;
	}
#endif

	if (higher_id == -1){
		//fist agent in the chain, disallow cyclic movments by preventing other agents moving to first agent's previous location
		occupied[prev_loc] = true;
	}

	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<MY_PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));

		int min_heuristic = get_gp_h(lns, curr_id, neighbor);

		successors.emplace_back(neighbor,min_heuristic,-1,rand());
	}

	int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);

	successors.emplace_back(prev_loc, wait_heuristic,-1,rand());



#if USE_WALL_HUGGING && USE_WALL_HUGGING_PIBT
    // Compute wall-hugging data for this agent
    int corridor_dir = 0;
    if (CenterlineAccess::g_branch_dist && CenterlineAccess::g_main_dist &&
        CenterlineAccess::g_zone_is_main && CenterlineAccess::g_n24c > 0) {
        int a_zi = (prev_loc / lns.env->cols / 24) * CenterlineAccess::g_n24c + (prev_loc % lns.env->cols / 24);
        int g_zi = (target / lns.env->cols / 24) * CenterlineAccess::g_n24c + (target % lns.env->cols / 24);
        if (a_zi >= 0 && a_zi < (int)CenterlineAccess::g_zone_is_main->size() &&
            (*CenterlineAccess::g_zone_is_main)[a_zi] &&
            g_zi >= 0 && g_zi < (int)CenterlineAccess::g_zone_is_main->size() &&
            (*CenterlineAccess::g_zone_is_main)[g_zi] && g_zi != a_zi) {
            int a_m = (*CenterlineAccess::g_main_dist)[prev_loc];
            int g_m = (*CenterlineAccess::g_main_dist)[target];
            if (a_m >= 0 && g_m >= 0) {
                corridor_dir = (g_m > a_m) ? 1 : -1;
            }
        }
    }
#endif

#if USE_TRAFFIC_PREDICTION
    // Narrow corridor entry check: ONLY when agent's NEXT STEP enters a narrow cell
    if (CenterlineAccess::g_segments_ptr && CenterlineAccess::g_main_dist) {
        int a_m = (*CenterlineAccess::g_main_dist)[prev_loc];
        if (a_m >= 0) {
            int task_m = lns.tasks.at(curr_id);
            int dir = (task_m > a_m) ? +1 : -1;
            typedef Centerline::TrafficSegment Seg;
            const std::vector<Seg>* segs = (const std::vector<Seg>*)CenterlineAccess::g_segments_ptr;
            if (segs) {
                // Check if ANY successor location enters a blocked narrow segment
                for (auto& succ : successors) {
                    int s_m = (*CenterlineAccess::g_main_dist)[succ.location];
                    if (s_m < 0) continue;
                    for (auto& seg : *segs) {
                        if (!seg.is_narrow) continue;
                        int sm_lo = seg.si_lo < (int)CenterlineAccess::g_mp_main_dist->size() ?
                            (*CenterlineAccess::g_mp_main_dist)[seg.si_lo] : 999999;
                        int sm_hi = (seg.si_hi > 0 && seg.si_hi-1 < (int)CenterlineAccess::g_mp_main_dist->size()) ?
                            (*CenterlineAccess::g_mp_main_dist)[seg.si_hi-1] : 999999;
                        // succ is inside narrow segment, current is NOT inside
                        bool entering = (s_m >= sm_lo && s_m <= sm_hi) && 
                                        !(a_m >= sm_lo && a_m <= sm_hi);
                        if (entering && seg.flow_dir != 0 && dir != 0 && dir != seg.flow_dir) {
                            // Penalize this successor (but don't force WAIT, just prefer other directions)
                            succ.heuristic += 99999;
                        }
                    }
                }
            }
        }
    }
#endif

	std::sort(successors.begin(), successors.end(), 
		[&](MY_PIBT_C& a, MY_PIBT_C& b)
		{
			int diff[4] = {1,lns.env->cols,-1,-lns.env->cols};
			if (a.heuristic == b.heuristic){
					//tie break on prefer moving forward
					if (a.location==orien_next_v && b.location!=orien_next_v)
						return true;
					if (a.location!=orien_next_v && b.location==orien_next_v)
						return false;

#if USE_WALL_HUGGING && USE_WALL_HUGGING_PIBT
                    // Wall-hugging: prefer side closer to own wall
                    if (corridor_dir != 0) {
                        int ba = (*CenterlineAccess::g_branch_dist)[a.location];
                        int bb = (*CenterlineAccess::g_branch_dist)[b.location];
                        ba *= corridor_dir;
                        bb *= corridor_dir;
                        if (ba != bb) return ba > bb;
                    }
#endif
                    // random tie break
                    return a.tie_breaker < b.tie_breaker;

			}
			return a.heuristic < b.heuristic; 
		});


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
            if (!causalMY_PIBT(lower_id,curr_id,prev_states,next_states, prev_decision,decision, occupied,lns)){
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

