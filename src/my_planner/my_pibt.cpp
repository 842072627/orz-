


#include "my_pibt.h"
#include "../pibt_lock.h"
#include "../lane_enforcer.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>

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
#if USE_CONGESTION_AVOIDANCE
	// ---- Congestion avoidance (lock_dir detection) ----
	float congestion_penalty = 0.0f;
	int cg_opp_main = -1;
	uint16_t cg_cw = 0;
	int cg_agents_near = 0;
	int cg_my_m = -1, cg_t_m = -1;
	int cg_lo = 0, cg_hi = 0;

#if CONGESTION_CSV_INTERVAL > 0
	static FILE* g_cong_csv = nullptr;
	if (!g_cong_csv) {
		g_cong_csv = fopen("congestion_log.csv", "w");
		if (g_cong_csv) {
			fprintf(g_cong_csv, "tick,aid,my_m,target_m,opp_m,cw,agents_near,inside,penalty\n");
		}
	}
#endif

	if (CenterlineAccess::g_main_dist && CenterlineAccess::g_mp_lock_dir &&
		CenterlineAccess::g_mp_loc && CenterlineAccess::g_mp_main_dist &&
		CenterlineAccess::g_mp_count > 0 && CenterlineAccess::g_cell_width) {

		cg_my_m = (*CenterlineAccess::g_main_dist)[prev_loc];
		int tgt = lns.tasks.at(curr_id);
		cg_t_m = (tgt >= 0) ? (*CenterlineAccess::g_main_dist)[tgt] : -1;

		if (cg_my_m >= 0 && cg_t_m >= 0) {
			int cg_dir = (cg_t_m > cg_my_m) ? 1 : -1;
			cg_lo = std::min(cg_my_m, cg_t_m);
			cg_hi = std::max(cg_my_m, cg_t_m);

			static std::vector<int> s_cell_occ;
			static int s_cache_tick = -1;
			if (s_cache_tick != lns.env->curr_timestep) {
				s_cache_tick = lns.env->curr_timestep;
				s_cell_occ.assign(lns.env->map.size(), 0);
				for (int a = 0; a < lns.env->num_of_agents; a++) {
					int loc = prev_states[a].location;
					if (loc >= 0 && loc < (int)s_cell_occ.size())
						s_cell_occ[loc]++;
				}
			}

			for (int si = 0; si < CenterlineAccess::g_mp_count; si++) {
				int sm = (*CenterlineAccess::g_mp_main_dist)[si];
				if (sm < cg_lo || sm > cg_hi) continue;
				int ld = (*CenterlineAccess::g_mp_lock_dir)[si];
				if (ld == 0 || ld == cg_dir) continue;

				cg_opp_main = sm;
				int mp_loc = (*CenterlineAccess::g_mp_loc)[si];
				cg_cw = (*CenterlineAccess::g_cell_width)[mp_loc];

				int cols = lns.env->cols;
				int mx = mp_loc % cols, my = mp_loc / cols;
				int cr = CONGESTION_CHECK_RANGE;
				for (int dy = -cr; dy <= cr && cg_agents_near == 0; dy++) {
					for (int dx = -cr; dx <= cr && cg_agents_near == 0; dx++) {
						int cx = mx + dx, cy = my + dy;
						if (cx < 0 || cx >= cols || cy < 0 || cy >= lns.env->rows) continue;
						if (s_cell_occ[cy * cols + cx] > 0)
							cg_agents_near++;
					}
				}
				break;
			}
		}
	}

	bool cg_active = (cg_opp_main >= 0 && cg_agents_near > 0 &&
					   cg_cw != 0xFFFF && cg_cw < 6);

	if (cg_active) {
		bool inside_corridor = (cg_my_m >= cg_lo && cg_my_m <= cg_hi);
		float cw_factor = (6.0f - (float)cg_cw) / 5.0f;

		if (inside_corridor) {
			congestion_penalty = cw_factor * (float)cg_agents_near * CONGESTION_INNER_WEIGHT;
		} else {
			int dist = std::abs(cg_my_m - cg_opp_main);
			float prox = 1.0f - (float)dist / (float)CONGESTION_LOOK_AHEAD;
			if (prox > 0)
				congestion_penalty = cw_factor * CONGESTION_OUTER_WEIGHT * prox;
		}
	}

#if CONGESTION_CSV_INTERVAL > 0
	if (g_cong_csv && congestion_penalty > 0) {
		int t = lns.env->curr_timestep;
		if (t % CONGESTION_CSV_INTERVAL == 0) {
			bool inside = (cg_my_m >= cg_lo && cg_my_m <= cg_hi);
			fprintf(g_cong_csv, "%d,%d,%d,%d,%d,%d,%d,%d,%.2f\n",
					t, curr_id, cg_my_m, cg_t_m,
					cg_opp_main, (int)cg_cw, cg_agents_near,
					inside ? 1 : 0, congestion_penalty);
			fflush(g_cong_csv);
		}
	}
#endif

	// Apply congestion penalty to successor computation below
#endif


	// for each neighbor of (prev_loc,prev_direction), and a wait copy of current location, generate a successor
	std::vector<int> neighbors;
	std::vector<MY_PIBT_C> successors;
	getNeighborLocs(&(lns.neighbors),neighbors,prev_loc);
	
#if USE_LANE_ENFORCEMENT
	// Compute agent travel direction for lane enforcement
	int lane_travel_dir = 0;
	int lane_tgt = lns.tasks.at(curr_id);
	if (lane_tgt >= 0 && CenterlineAccess::g_main_dist) {
		int cur_m = (*CenterlineAccess::g_main_dist)[prev_loc];
		int tgt_m = (*CenterlineAccess::g_main_dist)[lane_tgt];
		if (cur_m >= 0 && tgt_m >= 0)
			lane_travel_dir = (tgt_m > cur_m) ? 1 : -1;
	}
#endif

	for (auto& neighbor: neighbors){

		assert(validateMove(prev_loc, neighbor, lns.env));

#if USE_LANE_ENFORCEMENT
		if (lane_travel_dir != 0 && !LaneEnforcer::can_enter(neighbor, lane_travel_dir))
			continue;
#endif

		int min_heuristic = get_gp_h(lns, curr_id, neighbor);

		#if USE_CONGESTION_AVOIDANCE
		float succ_cong = 0.0f;
		if (cg_active) {
			int n_m = (*CenterlineAccess::g_main_dist)[neighbor];
			if (n_m >= 0) {
				int nd = std::abs(n_m - cg_opp_main);
				int od = std::abs(cg_my_m - cg_opp_main);
				if (nd <= od) succ_cong = congestion_penalty;
			}
		}
		successors.emplace_back(neighbor,min_heuristic,-1,rand(), succ_cong);
		#else
		successors.emplace_back(neighbor,min_heuristic,-1,rand());
		#endif
	}

	int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);

#if USE_CONGESTION_AVOIDANCE
	float wait_cong = 0.0f;
	if (cg_active) {
		int p_m = (*CenterlineAccess::g_main_dist)[prev_loc];
		if (p_m >= 0) {
			int cd = std::abs(p_m - cg_opp_main);
			int od = std::abs(cg_my_m - cg_opp_main);
			if (cd < od) wait_cong = congestion_penalty;
		}
	}
	successors.emplace_back(prev_loc, wait_heuristic,-1,rand(), wait_cong);
#else
	successors.emplace_back(prev_loc, wait_heuristic,-1,rand());
#endif



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
	std::sort(successors.begin(), successors.end(), 
		[&](MY_PIBT_C& a, MY_PIBT_C& b)
		{
			int diff[4] = {1,lns.env->cols,-1,-lns.env->cols};
#if USE_CONGESTION_AVOIDANCE
			float sa = (float)a.heuristic + a.congestion;
			float sb = (float)b.heuristic + b.congestion;
			if (sa == sb){
#else
			if (a.heuristic == b.heuristic){
#endif
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
#if USE_CONGESTION_AVOIDANCE
			return sa < sb;
#else
			return a.heuristic < b.heuristic; 
#endif
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

