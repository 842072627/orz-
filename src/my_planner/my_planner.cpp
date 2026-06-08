// Default planner baseline implementation.
//
// For each planning episode, this module updates guide paths with a bounded
// traffic-flow phase (Frank-Wolfe optimisation), then runs MY_PIBT to produce
// a multi-step plan. Internal rollout builds future steps; the environment
// snapshot is restored after planning.
//
// References:
//   Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024). Traffic flow
//   optimisation for lifelong multi-agent path finding. AAAI Conference on
//   Artificial Intelligence, Vol. 38, No. 18, pp. 20674-20682.
//   https://ojs.aaai.org/index.php/AAAI/article/view/30054/31856
//
//   Okumura, K., et al. (2022). Priority Inheritance with Backtracking (MY_PIBT)
//   for iterative multi-agent path finding. Artificial Intelligence, Vol. 310.

#include "my_planner.h"
#include "my_heuristics.h"
#include "SharedEnv.h"
#include "my_pibt.h"
#include "my_flow.h"
#include "my_const.h"
#include "../lane_cells.h"
#include "../pibt_lock.h"
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>


namespace MyPlanner{

    static const char* debug_action_to_string(Action action)
    {
        switch (action)
        {
            case Action::FW: return "FW";
            case Action::CR: return "CR";
            case Action::CCR: return "CCR";
            case Action::W: return "W";
            case Action::NA: return "NA";
            default: return "UNKNOWN";
        }
    }

    //default planner data
    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    TrajLNS trajLNS;
    std::mt19937 mt1;
}

// Lane flow gate state (outside MyPlanner namespace)
int LaneFlow::g_lane_lock_dir[LANE_TABLE_SIZE] = {0};
int LaneFlow::g_lane_next_vote_tick = 0;
int LaneFlow::g_lane_blocked_plus[LANE_TABLE_SIZE] = {0};
int LaneFlow::g_lane_blocked_minus[LANE_TABLE_SIZE] = {0};
int LaneFlow::g_lane_passed_plus[LANE_TABLE_SIZE] = {0};
int LaneFlow::g_lane_passed_minus[LANE_TABLE_SIZE] = {0};
// Exit target used by dummy goal system (independent of red light)
int* LaneFlow::g_lane_exit_target = nullptr;
bool LaneFlow::g_lane_exit_target_initialized = false;

// Lane congestion relief state
#if LANE_RELIEF_ENABLED
int LaneFlow::g_lane_relief_until[LANE_TABLE_SIZE] = {0};
int LaneFlow::g_lane_relief_dir[LANE_TABLE_SIZE] = {0};
#endif

namespace MyPlanner{

    // Agent direction from main line: +1=increasing main_dist, -1=decreasing, 0=unknown
    static inline int agent_main_dir(int curr_loc, int goal_loc) {
        if (!CenterlineAccess::g_main_dist) return 0;
        int cm = (*CenterlineAccess::g_main_dist)[curr_loc];
        int gm = (*CenterlineAccess::g_main_dist)[goal_loc];
        if (cm < 0 || gm < 0) return 0;
        return (gm > cm) ? 1 : -1;
    }

    // Scan path[0..lookahead] for first→last main_dist in given lane, return direction
    static int path_dir_in_lane(const std::vector<int>& path, int li, int lookahead) {
        if (!CenterlineAccess::g_main_dist) return 0;
        int first_m = -1, last_m = -1;
        int limit = std::min((int)path.size(), lookahead);
        for (int pi = 0; pi < limit; pi++) {
            int loc = path[pi];
            if (loc < 0) continue;
            if (get_cell_lane_fast(loc) != li) {
                if (first_m >= 0) break;  // exited lane, stop
                continue;
            }
            int m = (*CenterlineAccess::g_main_dist)[loc];
            if (m >= 0) {
                if (first_m < 0) first_m = m;
                last_m = m;
            }
        }
        if (first_m >= 0 && last_m >= 0 && first_m != last_m)
            return (last_m > first_m) ? 1 : -1;
        return 0;
    }

    // Scan guide path for lane entry (prev outside, curr inside), returns direction
#if USE_PLANNER_LANE_FLOW || LANE_RELIEF_ENABLED
    static int lane_entry_dir_short(const std::vector<int>& path, int cols, int goal_loc, int& out_lane) {
        out_lane = -1;
        if (path.size() < 2) return 0;
        int limit = std::min((int)path.size(), LANE_DIR_LOOKAHEAD + 1);
        for (int pi = 1; pi < limit; pi++) {
            int curr = path[pi];
            if (curr < 0) continue;
            int cl = get_cell_lane_fast(curr);
            if (cl >= 0 && get_cell_lane_fast(path[pi-1]) < 0) {
                out_lane = cl;
                // Found entry at curr. Continue for exit from this lane.
                int exit_loc = curr;
                int ext_limit = std::min((int)path.size(), LANE_DIR_LOOKAHEAD + 1);
                for (int pj = pi + 1; pj < ext_limit; pj++) {
                    int c2 = path[pj];
                    if (c2 < 0) continue;
                    if (get_cell_lane_fast(c2) != cl) break;
                    exit_loc = c2;
                }
                // Direction = sign(main_dist[exit] - main_dist[entry])
                if (CenterlineAccess::g_main_dist) {
                    int em = (*CenterlineAccess::g_main_dist)[curr];
                    int xm = (*CenterlineAccess::g_main_dist)[exit_loc];
                    if (em >= 0 && xm >= 0)
                        return (xm > em) ? 1 : -1;
                }
                return agent_main_dir(curr, goal_loc);
            }
        }
        return 0;
    }

    // Traffic light vote: freeze lock if core zone occupied
    static void lane_vote(SharedEnvironment* env) {
        int ts = env->system_timestep >= 0 ? env->system_timestep : env->curr_timestep;
        if (ts < LaneFlow::g_lane_next_vote_tick) return;
        LaneFlow::g_lane_next_vote_tick = ts + LANE_VOTE_PERIOD;

        // Check each lane's core zone for occupancy (only agents with real tasks)
        bool core_occupied[LANE_TABLE_SIZE] = {false};
        int core_count[LANE_TABLE_SIZE] = {0};  // number of agents in core
        for (int i = 0; i < env->num_of_agents; i++) {
            if (env->curr_task_schedule[i] < 0) continue;  // no real task
            int loc = env->start_states[i].location;
            if (loc < 0) continue;
            int cl = get_cell_core_fast(loc);
            if (cl >= 0 && cl < LANE_TABLE_SIZE) {
                core_occupied[cl] = true;
                core_count[cl]++;
            }
        }

        int cnt0[LANE_TABLE_SIZE] = {0}, cnt1[LANE_TABLE_SIZE] = {0};
        for (int i = 0; i < env->num_of_agents; i++) {
            if (env->curr_task_schedule[i] < 0) continue;  // no real task
            int loc = env->start_states[i].location;
            if (loc < 0) continue;
            int li = get_cell_lane_fast(loc);
            if (li < 0 || li >= LANE_TABLE_SIZE) continue;
            int dir = path_dir_in_lane(trajLNS.trajs[i], li, LANE_DIR_LOOKAHEAD);
            if (dir > 0) cnt0[li]++;
            else if (dir < 0) cnt1[li]++;
        }

        int prev_lock[LANE_TABLE_SIZE];
        for (int li = 0; li < LANE_TABLE_SIZE; li++) {
            prev_lock[li] = LaneFlow::g_lane_lock_dir[li];
        }
        for (int li = 0; li < LANE_TABLE_SIZE; li++) {
#if USE_LANE_FLOW_STATS
            bool flow_exceeded = false;
            if (prev_lock[li] != 0) {
                int blocked = (prev_lock[li] > 0)
                    ? LaneFlow::g_lane_blocked_minus[li]
                    : LaneFlow::g_lane_blocked_plus[li];
                int passed = (prev_lock[li] > 0)
                    ? LaneFlow::g_lane_passed_plus[li]
                    : LaneFlow::g_lane_passed_minus[li];
                if (blocked > passed * LANE_FLOW_RATIO_FLIP)
                    flow_exceeded = true;
            }
#endif
            if (core_occupied[li] && core_count[li] > LANE_CORE_CLEAR_THRESH) {
                // 核心区 > 4人 → 冻锁
                LaneFlow::g_lane_lock_dir[li] = prev_lock[li];
            } else if (flow_exceeded) {
                // 核心区 <= 4人 + 流比异常 → 强制翻锁
                LaneFlow::g_lane_lock_dir[li] = -prev_lock[li];
                if (LaneFlow::g_lane_lock_dir[li] == 0)
                    LaneFlow::g_lane_lock_dir[li] = 1;
            } else {
                // 核心区 <= 4人 + 流比正常 → cnt0/cnt1 投票
                if (cnt0[li] == 0 && cnt1[li] == 0) {
                    LaneFlow::g_lane_lock_dir[li] = 0;
                } else if (cnt1[li] == 0) {
                    LaneFlow::g_lane_lock_dir[li] = 1;
                } else if (cnt0[li] == 0) {
                    LaneFlow::g_lane_lock_dir[li] = -1;
                } else {
                    LaneFlow::g_lane_lock_dir[li] = -(prev_lock[li]);
                    if (LaneFlow::g_lane_lock_dir[li] == 0)
                        LaneFlow::g_lane_lock_dir[li] = 1;
                }
            }
        }
        // CSV debug: log per-lane lock state and flow stats
        static FILE* lf = nullptr;
        if (!lf) {
            lf = fopen("lane_gate_debug.csv", "w");
            if (lf) fprintf(lf, "tick,li,lock,cnt0,cnt1,core_occ,workers_in_lane,workers_in_core,blk_p,blk_m,pass_p,pass_m\n");
        }
        if (lf && (ts % 400 == 0 || ts <= 2000)) {
            for (int li = 0; li < LANE_TABLE_SIZE; li++) {
                int workers_in_lane = 0, workers_in_core = 0;
                for (int i = 0; i < env->num_of_agents; i++) {
                    if (env->curr_task_schedule[i] < 0) continue;
                    int loc = env->start_states[i].location;
                    if (loc < 0) continue;
                    if (get_cell_lane_fast(loc) == li) workers_in_lane++;
                    if (get_cell_core_fast(loc) == li) workers_in_core++;
                }
                fprintf(lf, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    ts, li, LaneFlow::g_lane_lock_dir[li],
                    cnt0[li], cnt1[li], core_occupied[li]?1:0,
                    workers_in_lane, workers_in_core,
                    LaneFlow::g_lane_blocked_plus[li],
                    LaneFlow::g_lane_blocked_minus[li],
                    LaneFlow::g_lane_passed_plus[li],
                    LaneFlow::g_lane_passed_minus[li]);
            }
            fflush(lf);
        }
#if USE_LANE_FLOW_STATS
        // Reset per-period counters
        for (int li = 0; li < LANE_TABLE_SIZE; li++) {
            LaneFlow::g_lane_blocked_plus[li] = 0;
            LaneFlow::g_lane_blocked_minus[li] = 0;
            LaneFlow::g_lane_passed_plus[li] = 0;
            LaneFlow::g_lane_passed_minus[li] = 0;
        }
#endif
    }

    // Apply lane priority delta from main line direction
    static void apply_lane_priority(std::vector<double>& priority, SharedEnvironment* env) {
        for (int i = 0; i < env->num_of_agents; i++) {
            int li = -1;
            int dir = lane_entry_dir_short(trajLNS.trajs[i], env->cols, trajLNS.tasks[i], li);
            if (dir == 0 || li < 0) continue;
            int lock = LaneFlow::g_lane_lock_dir[li];
            if (lock == 0) continue;
            if (dir == lock) priority[i] += LANE_PRIORITY_DELTA;
            else priority[i] -= LANE_PRIORITY_DELTA;
        }
    }
#endif  // USE_PLANNER_LANE_FLOW || LANE_RELIEF_ENABLED

    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
    {
        State next = state;
        next.timestep = state.timestep + 1;

        if (action == Action::CR)
        {
            next.orientation = (state.orientation + 1) % 4;
            return next;
        }
        if (action == Action::CCR)
        {
            next.orientation = (state.orientation + 3) % 4;
            return next;
        }
        if (action != Action::FW)
        {
            return next;
        }

        int delta = 0;
        if (state.orientation == 0) delta = 1;
        if (state.orientation == 1) delta = env->cols;
        if (state.orientation == 2) delta = -1;
        if (state.orientation == 3) delta = -env->cols;

        int next_loc = state.location + delta;
        if (next_loc >= 0 && next_loc < env->map.size() && validateMove(state.location, next_loc, env))
        {
            next.location = next_loc;
        }
        return next;
    }

    // BFS: compute nearest non-lane exit target for every lane cell
    static void init_lane_exit_targets(SharedEnvironment* env) {
        if (LaneFlow::g_lane_exit_target_initialized) return;
        int N = env->map.size();
        if (!LaneFlow::g_lane_exit_target)
            LaneFlow::g_lane_exit_target = new int[N];
        std::fill(LaneFlow::g_lane_exit_target, LaneFlow::g_lane_exit_target + N, -1);
        std::queue<int> q;
        for (int loc = 0; loc < N; loc++) {
            if (get_cell_lane_fast(loc) < 0 && env->map[loc] == 0) {
                LaneFlow::g_lane_exit_target[loc] = loc;
                q.push(loc);
            }
        }
        const int d4[4] = {1, -1, env->cols, -env->cols};
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int cur_t = LaneFlow::g_lane_exit_target[cur];
            for (int di = 0; di < 4; di++) {
                int nxt = cur + d4[di];
                if (nxt < 0 || nxt >= N) continue;
                if (env->map[nxt] != 0) continue;  // wall
                if (LaneFlow::g_lane_exit_target[nxt] >= 0) continue;  // already set
                LaneFlow::g_lane_exit_target[nxt] = cur_t;
                q.push(nxt);
            }
        }
        LaneFlow::g_lane_exit_target_initialized = true;
    }

    static void initialize_dummy_goals_if_needed(SharedEnvironment* env)
    {
#if USE_DUMMY_GOAL_STAY
#if USE_DUMMY_EXIT_TARGET
        // Dummy goal = nearest non-lane exit target → idle agent exits lane and stays put
        init_lane_exit_targets(env);
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++) {
            int loc = env->start_states[i].location;
            int target = (loc >= 0 && loc < env->map.size())
                ? LaneFlow::g_lane_exit_target[loc] : -1;
            dummy_goals.at(i) = (target >= 0) ? target : loc;
        }
#else
        // Dummy goal = agent's current position → idle agent stays exactly where it is
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
            dummy_goals.at(i) = env->start_states.at(i).location;
#endif
#else
        // Original: only at tick 0, dummy goal = spawn position → idle agent walks home
        int ts = env->system_timestep >= 0 ? env->system_timestep : env->curr_timestep;
        if (ts != 0)
            return;
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
            dummy_goals.at(i) = env->start_states.at(i).location;
#endif
    }

    static void setup_multistep_episode_state(SharedEnvironment* env, TimePoint flow_end_time,
                                              std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        for(int i=0; i<env->num_of_agents; i++)
        {
            // initialise heuristic tables for goals
            if (std::chrono::steady_clock::now() < flow_end_time){
                for (int j = 0; j < env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.find(goal_loc) == trajLNS.heuristics.end() ||
                        trajLNS.heuristics[goal_loc].empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
                    }
                }
            }

            // set goals/tasks
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // detect guide-path mismatch (goal changed or no guide path)
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                require_guide_path[i] = true;

            // update per-agent transient planning state
            assert(env->start_states[i].location >=0);
            prev_states[i] = env->start_states[i];
            next_states[i] = State();
            prev_decision[env->start_states[i].location] = i;
            if (decided[i].loc == -1){
                decided[i].loc = env->start_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            // cross-episode priority update
            if(require_guide_path[i])
                local_priority[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                local_priority[i] = local_priority[i]+1;

            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->start_states[i].location].size() == 1){
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }

    static void update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
    {
        // one-time guide-path update for goal-changed agents
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (std::chrono::steady_clock::now() > flow_end_time)
                break;
            if (require_guide_path[i]){
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &flow_end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()){
                        add_traj(trajLNS, i);
                    }
                    break;
                }
            }
        }

        // one-time FW optimization in the flow allocation phase
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated, flow_end_time);
    }

    static void refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        for(int i=0; i<env->num_of_agents; i++)
        {
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
                local_priority[i] = local_priority[i] + 1;
            }

            assert(env->start_states[i].location >=0);
            prev_states[i] = env->start_states[i];
            next_states[i] = State();
            prev_decision[env->start_states[i].location] = i;

            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->start_states[i].location].size() == 1){
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }

    // ==== 连续空间 swept-rect 碰撞检测辅助函数 ====
    static bool swept_interval(float d0, float dv, float lower, float upper,
                                float& out_t0, float& out_t1) {
        const float eps = 1e-6f;
        if (dv > -eps && dv < eps) {
            if (d0 > lower + eps && d0 < upper - eps) {
                out_t0 = 0.0f; out_t1 = 1.0f;
                return true;
            }
            return false;
        }
        float t0 = (lower - d0) / dv;
        float t1 = (upper - d0) / dv;
        if (t0 > t1) std::swap(t0, t1);
        const float teps = eps / std::fabs(dv);
        t0 += teps; t1 -= teps;
        out_t0 = std::max(0.0f, t0);
        out_t1 = std::min(1.0f, t1);
        return out_t0 <= out_t1;
    }

    // 检查两个 agent 的 FW 动作在连续空间中是否 swept-rect 碰撞
    // 输入：两个 agent 在 prev_states 和 next_states 中的位置
    // 返回 true 如果它们的 1x1 box 在 [0,1] 时间内碰撞
    static bool swept_collision_check(int i, int j,
        const std::vector<State>& prev_states,
        const std::vector<State>& next_states,
        int cols, int rows)
    {
        const float AGENT_SIZE = 1.0f;

        int pi_loc = prev_states[i].location;
        int pj_loc = prev_states[j].location;
        int ni_loc = next_states[i].location;
        int nj_loc = next_states[j].location;

        if (ni_loc < 0 || nj_loc < 0) return false;
        if (pi_loc == pj_loc) return true;  // 同格子出发 → 必然碰撞

        float xi0 = (float)(pi_loc % cols);
        float yi0 = (float)(pi_loc / cols);
        float xi1 = (float)(ni_loc % cols);
        float yi1 = (float)(ni_loc / cols);
        float xj0 = (float)(pj_loc % cols);
        float yj0 = (float)(pj_loc / cols);
        float xj1 = (float)(nj_loc % cols);
        float yj1 = (float)(nj_loc / cols);

        // 轴对齐 swept 包围盒预筛
        float ri_min_x = std::min(xi0, xi1);
        float ri_max_x = std::max(xi0, xi1) + AGENT_SIZE;
        float ri_min_y = std::min(yi0, yi1);
        float ri_max_y = std::max(yi0, yi1) + AGENT_SIZE;
        float rj_min_x = std::min(xj0, xj1);
        float rj_max_x = std::max(xj0, xj1) + AGENT_SIZE;
        float rj_min_y = std::min(yj0, yj1);
        float rj_max_y = std::max(yj0, yj1) + AGENT_SIZE;

        if (ri_min_x >= rj_max_x || ri_max_x <= rj_min_x ||
            ri_min_y >= rj_max_y || ri_max_y <= rj_min_y)
            return false;

        // 精确连续时间碰撞
        float d0x = xi0 - xj0;
        float d0y = yi0 - yj0;
        float dvx = (xi1 - xi0) - (xj1 - xj0);
        float dvy = (yi1 - yi0) - (yj1 - yj0);

        float tx0 = 0, tx1 = 0, ty0 = 0, ty1 = 0;
        bool cx = swept_interval(d0x, dvx, -AGENT_SIZE, AGENT_SIZE, tx0, tx1);
        bool cy = swept_interval(d0y, dvy, -AGENT_SIZE, AGENT_SIZE, ty0, ty1);
        return cx && cy && std::max(tx0, ty0) <= std::min(tx1, ty1);
    }

    static void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                        std::vector<Action>& one_step_actions)
    {
        const auto start_time = std::chrono::steady_clock::now();
        std::vector<double> pibt_priority = local_priority;  // copy for lane adjustments
#if USE_PLANNER_LANE_FLOW || LANE_RELIEF_ENABLED
        apply_lane_priority(pibt_priority, env);  // modify copy only
#endif
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return pibt_priority.at(a) > pibt_priority.at(b);
            }
        );
        //clear reset occupied
        std::fill(occupied.begin(), occupied.end(), false);

        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalMY_PIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }

        one_step_actions.resize(env->num_of_agents);
        for (int id : ids){
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;
            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }
            one_step_actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;
        }

        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && one_step_actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,one_step_actions,prev_decision);
            }
        }

    // ==== 连续空间碰撞过滤 ====
    // 只检测对向流（同一轴线上相反方向）且距离 ≤ 2 的 FW agent 对
    {
        int cols = env->cols;

        // 收集每个 FW agent 的运动方向
        struct DirVec { int dx, dy; };
        std::vector<DirVec> fw_dir(env->num_of_agents, {0, 0});
        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int pi = prev_states[i].location;
            int ni = next_states[i].location;
            if (ni < 0) continue;
            fw_dir[i].dx = (ni % cols) - (pi % cols);
            fw_dir[i].dy = (ni / cols) - (pi / cols);
        }

        // 按 next_state 位置分 bin（只放 FW agent）
        std::unordered_map<int, std::vector<int>> bins;
        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int nloc = next_states[i].location;
            if (nloc >= 0) bins[nloc].push_back(i);
        }

        // 每个 agent 查周围 3x3 cell
        static const int dr[9] = {0, -1, 1, 0, 0, -1, -1, 1, 1};
        static const int dc[9] = {0, 0, 0, -1, 1, -1, 1, -1, 1};

        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int ni = next_states[i].location;
            if (ni < 0) continue;
            int ri = ni / cols, ci = ni % cols;
            int dxi = fw_dir[i].dx, dyi = fw_dir[i].dy;

            for (int d = 0; d < 9; d++) {
                int nr = ri + dr[d], nc = ci + dc[d];
                if (nr < 0 || nr >= env->rows || nc < 0 || nc >= cols) continue;
                auto it = bins.find(nr * cols + nc);
                if (it == bins.end()) continue;

                for (int j : it->second) {
                    if (j <= i) continue;
                    if (one_step_actions[j] != Action::FW) continue;
                    if (next_states[j].location < 0) continue;

                    // === 方向性预筛：只处理对向流 ===
                    int dxj = fw_dir[j].dx, dyj = fw_dir[j].dy;

                    // 两 agent 必须在同一轴线上且方向相反
                    // 即 (dxi * dxj < 0 && dyi == 0 && dyj == 0) 水平对向
                    // 或 (dyi * dyj < 0 && dxi == 0 && dxj == 0) 垂直对向
                    bool head_on = false;
                    if (dxi != 0 && dxj != 0 && dyi == 0 && dyj == 0 && dxi * dxj < 0)
                        head_on = true;
                    else if (dyi != 0 && dyj != 0 && dxi == 0 && dxj == 0 && dyi * dyj < 0)
                        head_on = true;

                    if (!head_on) continue;

                    // === 距离预筛：在运动方向上的距离 ≤ 2 ===
                    int pi_loc = prev_states[i].location;
                    int pj_loc = prev_states[j].location;
                    int dist = 999;
                    if (dxi != 0)  // 水平对向
                        dist = std::abs(pi_loc % cols - pj_loc % cols);
                    else  // 垂直对向
                        dist = std::abs(pi_loc / cols - pj_loc / cols);
                    if (dist > 2) continue;

                    // swept-rect 检查（与 ActionModel 相同逻辑）
                    if (!swept_collision_check(i, j, prev_states, next_states, cols, env->rows))
                        continue;

                    // 碰撞！低 priority 的 wait
                    int loser, winner;
                    if (local_priority[i] < local_priority[j]) {
                        loser = i; winner = j;
                    } else {
                        loser = j; winner = i;
                    }

                    one_step_actions[loser] = Action::W;
                    int old_loc = next_states[loser].location;
                    if (old_loc >= 0 && old_loc < (int)decision.size())
                        decision[old_loc] = -1;
                    decided[loser].loc = prev_states[loser].location;
                    decided[loser].state = DONE::DONE;
                    next_states[loser] = prev_states[loser];
                }
            }
        }
    }

        prev_states = next_states;

        const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_time).count();
    }

    static void append_actions_and_rollout_states(SharedEnvironment* env,
                                                   std::vector<std::vector<Action>> & actions,
                                                   const std::vector<Action>& one_step_actions)
    {
        for (int aid = 0; aid < env->num_of_agents; aid++){
            actions[aid].push_back(one_step_actions[aid]);
        }

        std::vector<State> rolled_states(env->num_of_agents);
        for (int aid = 0; aid < env->num_of_agents; aid++){
            rolled_states[aid] = rollout_next_state(env->start_states[aid], one_step_actions[aid], env);
        }
        env->start_states = rolled_states;
        env->curr_timestep += 1;
    }

#if USE_CONGESTION_DATA
    // ===== Congestion data collection (per-tick aggregate + per-lane every 200 ticks) =====
    static std::vector<int> g_cd_prev_loc;
    static std::vector<int> g_cd_prev_in_lane;
    static std::vector<int> g_cd_stuck_ticks;
    static int g_cd_core_cell_count[LANE_TABLE_SIZE] = {0};
    static bool g_cd_inited = false;
    static int g_cd_last_lane_tick = 0;  // last tick we wrote per-lane data
    static int g_cd_call_count = 0;      // global call counter for periodic lane CSV
    static int g_cd_prev_lane_agents[LANE_TABLE_SIZE] = {0};  // previous call's lane agent counts
    static int g_cd_win_start_tick = -1; // start tick of current accumulation window
    static int g_cd_win_inflow[LANE_TABLE_SIZE] = {0};   // window-accumulated inflow
    static int g_cd_win_outflow[LANE_TABLE_SIZE] = {0};  // window-accumulated outflow
    static int g_cd_prev_win_outflow[LANE_TABLE_SIZE] = {0}; // completed window's outflow
    static int g_cd_real_last_tick = -1;  // last system tick processed for real-state tracking
    static std::vector<int> g_cd_real_prev_lane;  // [aid] = lane from real state at last plan() start
    static std::vector<bool> g_cd_agent_outflow_window; // [aid] = counted outflow this window (dedup)

    static void init_congestion_data(SharedEnvironment* env) {
        if (g_cd_inited) return;
        int N = env->num_of_agents;
        g_cd_prev_loc.assign(N, -1);
        g_cd_prev_in_lane.assign(N, -1);
        g_cd_stuck_ticks.assign(N, 0);
        for (int loc = 0; loc < (int)env->map.size(); loc++) {
            if (env->map[loc] != 0) continue;
            int cl = get_cell_core_fast(loc);
            if (cl >= 0 && cl < LANE_TABLE_SIZE)
                g_cd_core_cell_count[cl]++;
        }
        g_cd_inited = true;
    }

    static void collect_congestion_data(SharedEnvironment* env, int tick) {
        if (!g_cd_inited) init_congestion_data(env);
        g_cd_call_count++;
        int N = env->num_of_agents;
        int cols = env->cols;

        // Per-lane accumulators
        int lane_agents[LANE_TABLE_SIZE] = {0};
        int lane_idle_cnt[LANE_TABLE_SIZE] = {0};
        int core_agents_cnt[LANE_TABLE_SIZE] = {0};
        int core_idle_cnt[LANE_TABLE_SIZE] = {0};
        int core_stuck10[LANE_TABLE_SIZE] = {0};
        int core_stuck50[LANE_TABLE_SIZE] = {0};
        int core_stuck100[LANE_TABLE_SIZE] = {0};
        int dir_plus_cnt[LANE_TABLE_SIZE] = {0};
        int dir_minus_cnt[LANE_TABLE_SIZE] = {0};
        int inflow_cnt[LANE_TABLE_SIZE] = {0};
        int outflow_cnt[LANE_TABLE_SIZE] = {0};
        int entry_q_cnt[LANE_TABLE_SIZE] = {0};

        // Aggregate totals
        int tot_core_agents = 0, tot_core_idle = 0;
        int tot_stuck10 = 0, tot_stuck50 = 0, tot_stuck100 = 0;
        int tot_lane_agents = 0, tot_lane_idle = 0;
        int tot_inflow = 0, tot_outflow = 0, tot_entry_q = 0;
        int tot_dir_plus = 0, tot_dir_minus = 0;

        // ---- Real-state inflow/outflow tracking (once per plan() call) ----
        // Uses env->start_states which is the REAL game state at plan() start
        if (tick != g_cd_real_last_tick) {
            g_cd_real_last_tick = tick;
            if ((int)g_cd_real_prev_lane.size() < N) g_cd_real_prev_lane.resize(N, -1);
            for (int i = 0; i < N; i++) {
                if (env->curr_task_schedule[i] < -1) continue;
                int loc = env->start_states[i].location;
                if (loc < 0) continue;
                int prev_li = g_cd_real_prev_lane[i];
                int li = get_cell_lane_fast(loc);
                g_cd_real_prev_lane[i] = li;
                if (prev_li != li) {
                    if (li >= 0 && li < LANE_TABLE_SIZE) { inflow_cnt[li]++; tot_inflow++; }
                    if (prev_li >= 0 && prev_li < LANE_TABLE_SIZE) {
                        // Count outflow only once per agent per window (avoid boundary jitter)
                        if ((int)g_cd_agent_outflow_window.size() <= i)
                            g_cd_agent_outflow_window.resize(i + 1, false);
                        if (!g_cd_agent_outflow_window[i]) {
                            g_cd_agent_outflow_window[i] = true;
                            outflow_cnt[prev_li]++; tot_outflow++;
                        }
                    }
                }
            }
            // Accumulate real-state inflow/outflow into window counters
            if (g_cd_win_start_tick < 0) g_cd_win_start_tick = tick;
            for (int li = 0; li < LANE_TABLE_SIZE; li++) {
                g_cd_win_inflow[li] += inflow_cnt[li];
                g_cd_win_outflow[li] += outflow_cnt[li];
            }
        }

        for (int i = 0; i < N; i++) {
            if (env->curr_task_schedule[i] < -1) continue; // skip invalid
            int loc = env->start_states[i].location;
            if (loc < 0) continue;
            bool has_task = (env->curr_task_schedule[i] >= 0);

            int li = get_cell_lane_fast(loc);
            int ci = get_cell_core_fast(loc);

            // Stuck tracking
            if ((int)g_cd_prev_loc.size() <= i) {
                g_cd_prev_loc.resize(i + 1, -1);
                g_cd_prev_in_lane.resize(i + 1, -1);
                g_cd_stuck_ticks.resize(i + 1, 0);
            }
            if (g_cd_prev_loc[i] == loc)
                g_cd_stuck_ticks[i]++;
            else
                g_cd_stuck_ticks[i] = 0;
            g_cd_prev_loc[i] = loc;

            int prev_li = g_cd_prev_in_lane[i];
            g_cd_prev_in_lane[i] = li;

            // Lane agent counts
            if (li >= 0 && li < LANE_TABLE_SIZE) {
                lane_agents[li]++;
                tot_lane_agents++;
                if (!has_task) { lane_idle_cnt[li]++; tot_lane_idle++; }

                // Direction from guide path
                if (i < (int)trajLNS.trajs.size()) {
                    int dir = path_dir_in_lane(trajLNS.trajs[i], li, LANE_DIR_LOOKAHEAD);
                    if (dir > 0) { dir_plus_cnt[li]++; tot_dir_plus++; }
                    else if (dir < 0) { dir_minus_cnt[li]++; tot_dir_minus++; }
                }
            }

            // Core agent counts
            if (ci >= 0 && ci < LANE_TABLE_SIZE) {
                core_agents_cnt[ci]++;
                tot_core_agents++;
                if (!has_task) { core_idle_cnt[ci]++; tot_core_idle++; }
                int st = g_cd_stuck_ticks[i];
                if (st >= 100) { core_stuck100[ci]++; tot_stuck100++; }
                if (st >= 50)  { core_stuck50[ci]++; tot_stuck50++; }
                if (st >= 10)  { core_stuck10[ci]++; tot_stuck10++; }
            }

            // Entry queue: outside lane but adjacent to a lane cell
            if (li < 0) {
                int r = loc / cols, c = loc % cols;
                int ndirs[4] = {1, -1, cols, -cols};
                for (int d = 0; d < 4; d++) {
                    int nloc = loc + ndirs[d];
                    if (nloc < 0 || nloc >= (int)env->map.size()) continue;
                    // Bounds check for lateral moves
                    if (ndirs[d] == 1 || ndirs[d] == -1) {
                        int nc = (loc % cols) + (ndirs[d] == 1 ? 1 : -1);
                        if (nc < 0 || nc >= cols) continue;
                    }
                    int nl = get_cell_lane_fast(nloc);
                    if (nl >= 0 && nl < LANE_TABLE_SIZE) {
                        entry_q_cnt[nl]++;
                        tot_entry_q++;
                        break;
                    }
                }
            }
        }


        // Write aggregate CSV (every tick)
        static FILE* agg_f = nullptr;
        if (!agg_f) {
            agg_f = fopen("congestion_agg.csv", "w");
            if (agg_f) fprintf(agg_f, "tick,core_agents,core_idle,stuck10,stuck50,stuck100,"
                "lane_agents,lane_idle,inflow,outflow,entry_q,dir_plus,dir_minus\n");
        }
        if (agg_f) {
            fprintf(agg_f, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                tick, tot_core_agents, tot_core_idle,
                tot_stuck10, tot_stuck50, tot_stuck100,
                tot_lane_agents, tot_lane_idle,
                tot_inflow, tot_outflow, tot_entry_q,
                tot_dir_plus, tot_dir_minus);
            if (tick % 500 == 0) fflush(agg_f);
        }

        // Write per-lane CSV (every 100 game ticks, accumulated window data)
        if (tick - g_cd_win_start_tick >= 100) {
            static FILE* lane_f = nullptr;
            if (!lane_f) {
                lane_f = fopen("congestion_lane.csv", "w");
                if (lane_f) fprintf(lane_f, "tick,li,lane_agents,lane_idle,"
                    "core_agents,core_idle,core_cells,core_occ_pct,"
                    "stuck10,stuck50,stuck100,"
                    "dir_plus,dir_minus,"
                    "win_inflow,win_outflow,entry_q\n");
            }
            if (lane_f) {
                for (int li = 0; li < LANE_TABLE_SIZE; li++) {
                    int cc = g_cd_core_cell_count[li];
                    int occ_pct = (cc > 0) ? (core_agents_cnt[li] * 100 / cc) : 0;
                    fprintf(lane_f, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                        tick, li,
                        lane_agents[li], lane_idle_cnt[li],
                        core_agents_cnt[li], core_idle_cnt[li], cc, occ_pct,
                        core_stuck10[li], core_stuck50[li], core_stuck100[li],
                        dir_plus_cnt[li], dir_minus_cnt[li],
                        g_cd_win_inflow[li], g_cd_win_outflow[li], entry_q_cnt[li]);
                }
                fflush(lane_f);
            }
            // Save completed window outflow for COND3, then reset
            for (int li = 0; li < LANE_TABLE_SIZE; li++) {
                g_cd_prev_win_outflow[li] = g_cd_win_outflow[li];
                g_cd_win_inflow[li] = 0;
                g_cd_win_outflow[li] = 0;
            }
            g_cd_agent_outflow_window.assign(g_cd_agent_outflow_window.size(), false);
            g_cd_win_start_tick = tick;
        }

        // Congestion alert: check each lane against detection rule
        // Rule: balance>=0.3 AND [(occ>=75% && stuck10>=2) || (occ>=65% && laneAg_growth>=50%)]
        {
            static FILE* alert_f = nullptr;
            if (!alert_f) {
                alert_f = fopen("congestion_alerts.csv", "w");
                if (alert_f) fprintf(alert_f, "call,tick,li,occ_pct,stuck10,growth_pct,balance,trigger_rule\n");
            }
            if (alert_f) {
                for (int li = 0; li < LANE_TABLE_SIZE; li++) {
                    int occ_pct = (g_cd_core_cell_count[li] > 0)
                        ? (core_agents_cnt[li] * 100 / g_cd_core_cell_count[li]) : 0;
                    int total_dir = dir_plus_cnt[li] + dir_minus_cnt[li];
                    double balance = (total_dir > 5)
                        ? (std::min(dir_plus_cnt[li], dir_minus_cnt[li]) * 2.0 / total_dir)
                        : 0.0;
                    
                    // Compute growth rate from previous snapshot
                    int growth = 0;
                    if (g_cd_prev_lane_agents[li] > 0 && lane_agents[li] > g_cd_prev_lane_agents[li])
                        growth = (lane_agents[li] - g_cd_prev_lane_agents[li]) * 100 / g_cd_prev_lane_agents[li];
                    
                    bool triggered = false;
                    const char* rule = "";
                    if (balance >= 0.3) {
                        if (occ_pct >= 75 && core_stuck10[li] >= 2) {
                            triggered = true;
                            rule = "occ75+s10";
                        } else if (occ_pct >= 65 && growth >= 50) {
                            triggered = true;
                            rule = "occ65+growth";
                        } else if (occ_pct >= 50 && lane_agents[li] >= 80
                                   && g_cd_prev_win_outflow[li] <= 15
                                   && g_cd_win_start_tick >= 0) {
                            int min_dir = std::min(dir_plus_cnt[li], dir_minus_cnt[li]);
                            if (min_dir >= 3) {
                                int max_dir = std::max(dir_plus_cnt[li], dir_minus_cnt[li]);
                                if (max_dir >= min_dir * 8) {
                                    triggered = true;
                                    rule = "tidal8";
                                }
                            }
                        }
                    }
                    if (triggered) {
                        fprintf(alert_f, "%d,%d,%d,%d,%d,%d,%.2f,%s\n",
                            g_cd_call_count, tick, li, occ_pct, core_stuck10[li],
                            growth, balance, rule);
                    }
                    
                    // Save for next growth calculation
                    g_cd_prev_lane_agents[li] = lane_agents[li];
                }
                if (tick % 100 == 0) fflush(alert_f);
            }
        }
    }
#endif

// ===== Lane congestion relief: redirect counterflow + red light =====
#if LANE_RELIEF_ENABLED
// Pre-computed exit fringe cells for each lane+direction
static std::vector<int> g_lane_fringe[LANE_TABLE_SIZE][2]; // [li][0]=dir-1, [li][1]=dir+1
static int g_lane_wall_fringe[LANE_TABLE_SIZE][2][2]; // [li][dir][0]=-branch墙边, [1]=+branch墙边
static bool g_lane_fringe_ready = false;
static int g_lane_core_cell_count[LANE_TABLE_SIZE] = {0};  // core cells per lane

static void init_lane_fringe(SharedEnvironment* env) {
    if (g_lane_fringe_ready) return;
    if (!CenterlineAccess::g_main_dist) return;
    int N = env->map.size();
    int cols = env->cols;
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        g_lane_fringe[li][0].clear();
        g_lane_fringe[li][1].clear();
    }
    // Step 1: find min/max main_dist per lane + count core cells
    int min_m[LANE_TABLE_SIZE], max_m[LANE_TABLE_SIZE];
    memset(g_lane_core_cell_count, 0, sizeof(g_lane_core_cell_count));
    for (int li = 0; li < LANE_TABLE_SIZE; li++) { min_m[li] = 99999999; max_m[li] = -1; }
    for (int loc = 0; loc < N; loc++) {
        if (env->map[loc] != 0) continue;
        int li = get_cell_lane_fast(loc);
        if (li >= 0 && li < LANE_TABLE_SIZE) {
            int m = (*CenterlineAccess::g_main_dist)[loc];
            if (m >= 0) {
                if (m < min_m[li]) min_m[li] = m;
                if (m > max_m[li]) max_m[li] = m;
            }
        }
        int ci = get_cell_core_fast(loc);
        if (ci >= 0 && ci < LANE_TABLE_SIZE) g_lane_core_cell_count[ci]++;
    }
    // Step 2: for cells near min main_dist → fringe[-1], near max → fringe[+1]
    // Fringe = walkable cells OUTSIDE the lane adjacent to lane edge cells
    for (int loc = 0; loc < N; loc++) {
        if (env->map[loc] != 0) continue;
        int li = get_cell_lane_fast(loc);
        if (li < 0 || li >= LANE_TABLE_SIZE) continue;
        int m = (*CenterlineAccess::g_main_dist)[loc];
        if (m < 0) continue;
        int range = max_m[li] - min_m[li];
        if (range <= 0) continue;
        int fringe_idx = (m - min_m[li] >= range / 2) ? 1 : 0;
        int ndirs[4] = {1, -1, cols, -cols};
        for (int d = 0; d < 4; d++) {
            int nloc = loc + ndirs[d];
            if (nloc < 0 || nloc >= N) continue;
            if (ndirs[d] == 1 || ndirs[d] == -1) {
                int nc = (loc % cols) + (ndirs[d] == 1 ? 1 : -1);
                if (nc < 0 || nc >= cols) continue;
            }
            if (env->map[nloc] != 0) continue;
            if (get_cell_lane_fast(nloc) >= 0) continue;
            auto& vec = g_lane_fringe[li][fringe_idx];
            if (std::find(vec.begin(), vec.end(), nloc) == vec.end())
                vec.push_back(nloc);
        }
    }
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        for (int di = 0; di < 2; di++) {
            auto& vec = g_lane_fringe[li][di];
            std::sort(vec.begin(), vec.end());
            vec.erase(std::unique(vec.begin(), vec.end()), vec.end());
        }
    }
    // Step 3: per lane+direction, pick fringe cells at max/min branch (wall sides)
    memset(g_lane_wall_fringe, -1, sizeof(g_lane_wall_fringe));
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        for (int di = 0; di < 2; di++) {
            int best_neg = -1, best_pos = -1;
            int max_neg = -99999999, min_pos = 99999999;
            for (int fc : g_lane_fringe[li][di]) {
                if (!CenterlineAccess::g_branch_dist) break;
                int bd = (*CenterlineAccess::g_branch_dist)[fc];
                if (bd < 0 && bd > max_neg) { max_neg = bd; best_neg = fc; }
                if (bd >= 0 && bd < min_pos) { min_pos = bd; best_pos = fc; }
            }
            // Fallback: if branch data missing, use first/last fringe cell
            if (best_neg < 0 && !g_lane_fringe[li][di].empty()) best_neg = g_lane_fringe[li][di][0];
            if (best_pos < 0 && !g_lane_fringe[li][di].empty()) best_pos = g_lane_fringe[li][di].back();
            g_lane_wall_fringe[li][di][0] = best_neg;
            g_lane_wall_fringe[li][di][1] = best_pos;
        }
    }
    g_lane_fringe_ready = true;
}

// Determine顺流 direction for a lane: majority direction of agents inside it
static int lane_majority_dir(int li, SharedEnvironment* env) {
    int cnt_plus = 0, cnt_minus = 0;
    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->curr_task_schedule[i] < 0) continue;
        int loc = env->start_states[i].location;
        if (loc < 0) continue;
        if (get_cell_lane_fast(loc) != li) continue;
        int dir = agent_main_dir(loc, trajLNS.tasks[i]);   // 当前→task main轴方向
        if (dir > 0) cnt_plus++;
        else if (dir < 0) cnt_minus++;
    }
    if (cnt_plus > cnt_minus) return 1;
    if (cnt_minus > cnt_plus) return -1;
    return (rand() > RAND_MAX / 2) ? 1 : -1;
}

// Check congestion trigger for a lane (same rule as alert CSV)
static bool lane_is_congested(int li, int& occ_pct, int& stuck10, int& growth_pct,
                              int lane_agents, int prev_lane_agents,
                              int core_agents, int core_cells,
                              int dir_plus, int dir_minus) {
    if (core_cells <= 0) return false;
    occ_pct = core_agents * 100 / core_cells;
    
    // Condition 3 (tidal): checked before balance — extreme imbalance IS the condition
    {
        int min_dir = std::min(dir_plus, dir_minus);
        if (min_dir >= 3) {
            int max_dir = std::max(dir_plus, dir_minus);
            if (max_dir >= min_dir * 8 && occ_pct >= 50 && lane_agents >= 80
                && g_cd_prev_win_outflow[li] <= 15 && g_cd_win_start_tick >= 0)
            {
                growth_pct = -g_cd_prev_win_outflow[li];
                return true;
            }
        }
    }
    
    // Flow balance check (for COND1 and COND2 only)
    int total_dir = dir_plus + dir_minus;
    float balance = (total_dir > 5) ? (std::min(dir_plus, dir_minus) * 2.0f / total_dir) : 0.0f;
    if (balance < LANE_RELIEF_BALANCE) return false;
    
    // Condition 1: high occ + stuck
    if (occ_pct >= LANE_RELIEF_OCC_THRESH && stuck10 >= LANE_RELIEF_S10_THRESH)
        return true;
    
    // Condition 2: moderate occ + rapid growth
    if (prev_lane_agents > 0 && lane_agents > prev_lane_agents) {
        growth_pct = (lane_agents - prev_lane_agents) * 100 / prev_lane_agents;
        if (occ_pct >= LANE_RELIEF_ALT_OCC && growth_pct >= LANE_RELIEF_ALT_GROWTH)
            return true;
    }
    
    return false;
}
// Per-tick: relieve congested lanes (called from plan() after lane_vote)
static int g_relief_prev_lane_agents[LANE_TABLE_SIZE] = {0};
static int g_relief_window_agents[LANE_TABLE_SIZE] = {0};
static int g_relief_window_tick = -100;  // first window update at tick 0
static std::mt19937 g_relief_rng(42);
// Diagnostic counters for relief effectiveness
static int g_relief_trigger_count = 0;
static int g_relief_redirect_total = 0;
static int g_relief_triggers_per_lane[LANE_TABLE_SIZE] = {0};
static int g_relief_redirects_per_lane[LANE_TABLE_SIZE] = {0};
static bool g_relief_printed_end = false;

// Find nearest fringe cell (by main_dist) for an agent in the lane
static int find_nearest_fringe_in_set(int loc, const std::vector<int>& fringe) {
    if (fringe.empty()) return -1;
    if (!CenterlineAccess::g_main_dist) return fringe[0];
    int agent_m = (*CenterlineAccess::g_main_dist)[loc];
    if (agent_m < 0) return fringe[0];
    int best = fringe[0];
    int best_d = abs((*CenterlineAccess::g_main_dist)[best] - agent_m);
    for (int f : fringe) {
        int d = abs((*CenterlineAccess::g_main_dist)[f] - agent_m);
        if (d < best_d) { best_d = d; best = f; }
    }
    return best;
}

static void relieve_congestion(SharedEnvironment* env) {
    // Lazy init fallback: if fringe not ready (main_dist init delayed), retry
    if (!g_lane_fringe_ready) init_lane_fringe(env);
    int tick = env->system_timestep >= 0 ? env->system_timestep : env->curr_timestep;
    int N = env->num_of_agents;
    
    // === STEP 1: Check expired relief → unlock ===
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        if (LaneFlow::g_lane_relief_until[li] > 0 && tick > LaneFlow::g_lane_relief_until[li]) {
            LaneFlow::g_lane_lock_dir[li] = 0;
            LaneFlow::g_lane_relief_until[li] = 0;
            LaneFlow::g_lane_relief_dir[li] = 0;
        }
    }
    
    // === STEP 2: Per-tick lane stats for detection ===
    int lane_agents[LANE_TABLE_SIZE] = {0};
    int core_agents[LANE_TABLE_SIZE] = {0};
    int stuck10[LANE_TABLE_SIZE] = {0};
    int dir_plus[LANE_TABLE_SIZE] = {0};
    int dir_minus[LANE_TABLE_SIZE] = {0};
    
    // Track stuck ticks per agent (use static storage)
    static std::vector<int> relief_prev_loc;
    static std::vector<int> relief_stuck;
    relief_prev_loc.resize(N, -1);
    relief_stuck.resize(N, 0);
    
    for (int i = 0; i < N; i++) {
        int loc = env->start_states[i].location;
        if (loc < 0) continue;
        int li = get_cell_lane_fast(loc);
        int ci = get_cell_core_fast(loc);
        
        // Stuck tracking
        if (relief_prev_loc[i] == loc)
            relief_stuck[i]++;
        else
            relief_stuck[i] = 0;
        relief_prev_loc[i] = loc;
        
        if (li >= 0 && li < LANE_TABLE_SIZE) {
            lane_agents[li]++;
            int dir = agent_main_dir(loc, trajLNS.tasks[i]);   // 当前→task main轴方向
            if (dir > 0) dir_plus[li]++;
            else if (dir < 0) dir_minus[li]++;
        }
        if (ci >= 0 && ci < LANE_TABLE_SIZE) {
            core_agents[ci]++;
            if (relief_stuck[i] >= 10) stuck10[ci]++;
        }
    }
    
    // Rolling window update (every 100 ticks for growth rate)
    if (tick - g_relief_window_tick >= 100) {
        for (int li = 0; li < LANE_TABLE_SIZE; li++)
            g_relief_window_agents[li] = lane_agents[li];
        g_relief_window_tick = tick;
    }
    
    // === STEP 3: Detect + trigger relief ===
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        // Skip if already in relief
        if (LaneFlow::g_lane_relief_until[li] > 0) continue;
        
        int occ = 0, s10 = stuck10[li], growth = 0;
        bool congested = lane_is_congested(li, occ, s10, growth,
                                          lane_agents[li], g_relief_window_agents[li],
                                          core_agents[li], g_lane_core_cell_count[li],
                                          dir_plus[li], dir_minus[li]);
        if (!congested) continue;
        
        // Congestion confirmed! Determine顺流 direction
        int flood_dir = lane_majority_dir(li, env);
        if (flood_dir == 0) flood_dir = 1;
        
        // Get exit fringe for this direction
        int fringe_idx = (flood_dir > 0) ? 1 : 0;
        auto& fringe = g_lane_fringe[li][fringe_idx];
        if (fringe.empty()) continue;
        
        // Counterflow: redirect to nearest fringe cell
        int redirected = 0;
        for (int i = 0; i < N; i++) {
            int loc = env->start_states[i].location;
            if (loc < 0) continue;
            if (get_cell_lane_fast(loc) != li) continue;
            
            int dir = agent_main_dir(loc, trajLNS.tasks[i]);   // 当前→task main轴方向
            if (dir == flood_dir) continue; // 顺流方向 → 不动
            
            // Redirect to wall-side fringe (same branch sign as agent)
            int bd = CenterlineAccess::g_branch_dist ? (*CenterlineAccess::g_branch_dist)[loc] : 0;
            int side = (bd >= 0) ? 1 : 0;
            int target = g_lane_wall_fringe[li][fringe_idx][side];
            if (target < 0) continue;
            trajLNS.tasks[i] = target;
            require_guide_path[i] = true;
            redirected++;
        }
        
        if (redirected == 0) continue;
        
        // Diagnostic
        g_relief_trigger_count++;
        g_relief_redirect_total += redirected;
        g_relief_triggers_per_lane[li]++;
        g_relief_redirects_per_lane[li] += redirected;
        fprintf(stderr, "[DIAG] relief TRIGGER tick=%d li=%d laneAg=%d cntFlow=%d redirected=%d occ=%d%% s10=%d floodDir=%+d expire=%d\n",
                tick, li, lane_agents[li], dir_minus[li] + dir_plus[li], redirected,
                occ, stuck10[li], flood_dir, tick + LANE_RELIEF_TICKS);
        fflush(stderr);
        
        // Enable red light lock
        LaneFlow::g_lane_lock_dir[li] = flood_dir;
        LaneFlow::g_lane_relief_until[li] = tick + LANE_RELIEF_TICKS;
        LaneFlow::g_lane_relief_dir[li] = flood_dir;
    }
    
    // === STEP 4: During active relief, keep overriding counterflow agent goals ===
    for (int li = 0; li < LANE_TABLE_SIZE; li++) {
        if (LaneFlow::g_lane_relief_until[li] <= 0) continue;
        if (tick > LaneFlow::g_lane_relief_until[li]) continue;
        
        int flood_dir = LaneFlow::g_lane_relief_dir[li];
        int fringe_idx = (flood_dir > 0) ? 1 : 0;
        auto& fringe = g_lane_fringe[li][fringe_idx];
        if (fringe.empty()) continue;
        
        for (int i = 0; i < N; i++) {
            int loc = env->start_states[i].location;
            if (loc < 0) continue;
            if (get_cell_lane_fast(loc) != li) continue;
            
            int dir = agent_main_dir(loc, trajLNS.tasks[i]);   // 当前→task main轴方向
            if (dir == flood_dir) continue;
            
            // Redirect to wall-side fringe (same branch as agent)
            int bd = CenterlineAccess::g_branch_dist ? (*CenterlineAccess::g_branch_dist)[loc] : 0;
            int side = (bd >= 0) ? 1 : 0;
            int target = g_lane_wall_fringe[li][fringe_idx][side];
            if (target < 0) continue;
            trajLNS.tasks[i] = target;
            require_guide_path[i] = true;
        }
    }
    
    // Save lane counts for next tick's growth calculation
    for (int li = 0; li < LANE_TABLE_SIZE; li++)
        g_relief_prev_lane_agents[li] = lane_agents[li];
    
    // Final diagnostic at end of run
    // Print every 500 ticks to confirm relief is active
    if (tick > 0 && tick % 500 == 0) {
        fprintf(stderr, "[DIAG] relief PROGRESS tick=%d triggers=%d redirects=%d\n",
                tick, g_relief_trigger_count, g_relief_redirect_total);
        fflush(stderr);
    }
    // End-of-run summary (print once near end)
    if (!g_relief_printed_end && tick >= 2990) {
        g_relief_printed_end = true;
        fprintf(stderr, "[DIAG] relief END total=%d redirects=%d\n",
                g_relief_trigger_count, g_relief_redirect_total);
        for (int li = 0; li < LANE_TABLE_SIZE; li++) {
            if (g_relief_triggers_per_lane[li] > 0)
                fprintf(stderr, "[DIAG] relief END Lane%02d: triggers=%d redirects=%d\n",
                        li, g_relief_triggers_per_lane[li], g_relief_redirects_per_lane[li]);
        }
        fflush(stderr);
    }
}
#endif

    /**
     * @brief Default planner initialization
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
#if LANE_RELIEF_ENABLED
            // Pre-compute exit fringe and core cell counts (once)
            init_lane_fringe(env);
#endif
            return;
    };

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * @param num_steps number of steps to plan
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using MY_PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit, std::vector<std::vector<Action>> & actions,
                         SharedEnvironment* env, int num_steps){
        actions.clear();
        //cout<<"num of steps to plan "<<num_steps<<endl;
        if (env == nullptr || env->num_of_agents <= 0){
            return;
        }

        actions.resize(env->num_of_agents);
        if (num_steps <= 0){
            return;
        }

        const auto episode_start = std::chrono::steady_clock::now();
        const TimePoint episode_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit));
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        if (pibt_time <= 0){
            pibt_time = 1;
        }
        const int flow_budget_ms = std::max(0, time_limit - std::max(MIN_PIBT_TIME, pibt_time * num_steps) - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
        TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

        std::vector<double> local_priority = p;

        const std::vector<State> original_states = env->start_states;
        const int original_timestep = env->curr_timestep;
        const std::vector<DCR> original_decided = decided;

        // Fresh execution tracking for each multi-step planning episode:
        // no ongoing orientation-compensation actions are carried across episodes.
        decided.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

        // --- One-time setup for this planning episode ---
        initialize_dummy_goals_if_needed(env);
        setup_multistep_episode_state(env, flow_end_time, local_priority);
#if LANE_RELIEF_ENABLED
        // Congestion relief: redirect counterflow BEFORE guide path recalculation
        relieve_congestion(env);
#endif
        update_guide_paths_once_for_multistep(env, flow_end_time);

        const auto after_setup = std::chrono::steady_clock::now();
        const auto setup_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after_setup - episode_start).count();
        // commit only cross-episode priority update (internal rollout keeps using local_priority only)
        p = local_priority;

#if USE_PLANNER_LANE_FLOW
        // Periodic lane direction vote
        lane_vote(env);
#endif

        for (int step = 0; step < num_steps; step++){
            if (std::chrono::steady_clock::now() >= episode_deadline){
                break;
            }
            std::vector<Action> one_step_actions;
            // After step 0, only run MY_PIBT + rollout (no guide path recompute and no FW)
            if (step > 0){
                refresh_multistep_step_state(env, local_priority);
            }
            run_multistep_pibt_once(env, local_priority, one_step_actions);
            append_actions_and_rollout_states(env, actions, one_step_actions);
#if USE_CONGESTION_DATA
            collect_congestion_data(env, env->system_timestep);
#endif
        }

        // restore env state after internal rollout simulation
        env->start_states = original_states;
        env->curr_timestep = original_timestep;
        decided = original_decided;
    }
}
