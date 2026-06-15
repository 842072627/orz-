// optplan/planner.cpp — 优化版 Planner 实现
//
// 完整复制 DefaultPlanner::plan 的逻辑骨架，
// 维护自己的完整状态（TrajLNS、decision、priority 等）。

#include "planner.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "MapProfile.h"
#include "RuntimeConfig.h"
#include "lane_data.h"
#include "lane_traffic_light.h"
// OptPlan 命名空间下的副本（从 default_planner 复制，可自由修改）
#include "heuristics.h"
#include "flow.h"
#include "search.h"
#include "pibt.h"
#include "utils.h"
#include "const.h"

#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <queue>
#include <cstring>

using namespace OptPlan;

namespace optsvr {

// ============ rollout_next_state ============
State OptPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
{
    State next = state;
    next.timestep = state.timestep + 1;

    if (action == Action::CR) {
        next.orientation = (state.orientation + 1) % 4;
        return next;
    }
    if (action == Action::CCR) {
        next.orientation = (state.orientation + 3) % 4;
        return next;
    }
    if (action != Action::FW) {
        return next;
    }

    int delta = 0;
    if (state.orientation == 0) delta = 1;
    if (state.orientation == 1) delta = env->cols;
    if (state.orientation == 2) delta = -1;
    if (state.orientation == 3) delta = -env->cols;

    int next_loc = state.location + delta;
    if (next_loc >= 0 && next_loc < (int)env->map.size() && validateMove(state.location, next_loc, env)) {
        next.location = next_loc;
    }
    return next;
}

// ============ initialize ============
void OptPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    assert(env->num_of_agents != 0);

    p_.resize(env->num_of_agents);
    decision_.resize(env->map.size(), -1);
    prev_states_.resize(env->num_of_agents);
    next_states_.resize(env->num_of_agents);
    decided_.resize(env->num_of_agents, DCR({-1, DONE::DONE}));
    occupied_.resize(env->map.size(), false);
    checked_.resize(env->num_of_agents, false);
    ids_.resize(env->num_of_agents);
    require_guide_path_.resize(env->num_of_agents, false);
    last_guide_tick_.resize(env->num_of_agents, -1);  // -1 = 从未拿到过路径

    for (int i = 0; i < (int)ids_.size(); i++) {
        ids_[i] = i;
    }

    // 初始化全局启发式表（DefaultPlanner 的全局数据）
    init_heuristics(env);
    mt1_.seed(0);
    srand(0);

    // 使用 placement new 初始化 TrajLNS
    new (&trajLNS_) TrajLNS(env, global_heuristictable, global_neighbors);
    trajLNS_.init_mem();

    // 分配初始优先级
    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < (int)ids_.size(); i++) {
        p_[ids_[i]] = ((double)(ids_.size() - i)) / ((double)(ids_.size() + 1));
    }
    p_copy_ = p_;
    initialized_ = true;

    // 初始化 PIBT 密度惩罚系统
    pibt_density_.initialize(env);

    // Warehouse 通道定向策略
    if (cfg().ware_direct && map_profile().type == MapType::WAREHOUSE_DOMAIN) {
        build_row_column_highway(env);
    }
}

// ============ init_lane_exit_targets ============
// Two BFS from each side's non-lane walkable cells outward.
// g_exit_to_comp1[loc] = nearest non-lane cell on comp1 side for any cell
// g_exit_to_comp2[loc] = nearest non-lane cell on comp2 side for any cell
// Used so counterflow agents exit to the CORRECT side (not deeper into the lane).
static void init_lane_exit_targets(SharedEnvironment* env)
{
    if (IronLane::g_exit_target_initialized) return;
    int N = env->map.size();
    const int d4[4] = {1, -1, env->cols, -env->cols};

    // BFS from comp1 side (component 1): all non-lane walkable cells in comp1
    if (!IronLane::g_exit_to_comp1)
        IronLane::g_exit_to_comp1 = new int[N];
    std::fill(IronLane::g_exit_to_comp1, IronLane::g_exit_to_comp1 + N, -1);
    {
        std::queue<int> q;
        for (int loc = 0; loc < N; loc++) {
            if (!IronLane::is_lane_cell(loc) && env->map[loc] == 0
                && IronLane::get_component(loc) == 1) {
                IronLane::g_exit_to_comp1[loc] = loc;
                q.push(loc);
            }
        }
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int cur_t = IronLane::g_exit_to_comp1[cur];
            for (int di = 0; di < 4; di++) {
                int nxt = cur + d4[di];
                if (nxt < 0 || nxt >= N) continue;
                if (env->map[nxt] != 0) continue;
                if (IronLane::g_exit_to_comp1[nxt] >= 0) continue;
                IronLane::g_exit_to_comp1[nxt] = cur_t;
                q.push(nxt);
            }
        }
    }

    // BFS from comp2 side (component 2): all non-lane walkable cells in comp2
    if (!IronLane::g_exit_to_comp2)
        IronLane::g_exit_to_comp2 = new int[N];
    std::fill(IronLane::g_exit_to_comp2, IronLane::g_exit_to_comp2 + N, -1);
    {
        std::queue<int> q;
        for (int loc = 0; loc < N; loc++) {
            if (!IronLane::is_lane_cell(loc) && env->map[loc] == 0
                && IronLane::get_component(loc) == 2) {
                IronLane::g_exit_to_comp2[loc] = loc;
                q.push(loc);
            }
        }
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int cur_t = IronLane::g_exit_to_comp2[cur];
            for (int di = 0; di < 4; di++) {
                int nxt = cur + d4[di];
                if (nxt < 0 || nxt >= N) continue;
                if (env->map[nxt] != 0) continue;
                if (IronLane::g_exit_to_comp2[nxt] >= 0) continue;
                IronLane::g_exit_to_comp2[nxt] = cur_t;
                q.push(nxt);
            }
        }
    }

    IronLane::g_exit_target_initialized = true;
}

// ============ initialize_dummy_goals_if_needed ============
void OptPlanner::initialize_dummy_goals_if_needed(SharedEnvironment* env)
{
    if (env->curr_timestep != 0)
        return;
    if (optsvr::cfg().iron_lane_enabled) {
        // Iron lane: dummy goal = nearest non-lane exit target
        // idle agents will walk out of the lane instead of sitting on it
        init_lane_exit_targets(env);
        dummy_goals_.resize(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; i++) {
            int loc = env->curr_states[i].location;
            // Idle agent dummy goal:
            // - In lane: follow lock direction (same as counterflow exit side)
            // - Outside lane: stay at current position (don't wander)
            int exit = -1;
            if (loc >= 0 && loc < (int)env->map.size() && IronLane::is_lane_cell(loc)) {
                int lock = IronLane::g_lane_lock_dir;
                exit = (lock > 0)
                    ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                    : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
            }
            dummy_goals_[i] = (exit >= 0) ? exit : loc;
        }
    } else {
        dummy_goals_.resize(env->num_of_agents);
        for (int i = 0; i < env->num_of_agents; i++) {
            dummy_goals_[i] = env->curr_states[i].location;
        }
    }
}

// ============ setup_multistep_episode_state ============
void OptPlanner::setup_multistep_episode_state(
    SharedEnvironment* env, TimePoint flow_end_time,
    std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        // 设置 task
        if (env->goal_locations[i].empty()) {
            // 空闲 agent: 目标=dummy_goal（可能指向lane出口），极低优先级不影响主干道
            trajLNS_.tasks[i] = dummy_goals_.at(i);
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        // DIAG: log agent 8914 every tick for first 200 ticks
        if (i == 8914 && env->curr_timestep < 200 && optsvr::cfg().iron_lane_debug) {
            int loc = env->curr_states[i].location;
            int nb = (loc >= 0) ? static_cast<int>(trajLNS_.neighbors[loc].size()) : -1;
            bool has_goal = !env->goal_locations[i].empty();
            std::fprintf(stderr, "[Diag8914] tick=%d loc=%d pri=%.1f hasGoal=%d nb=%d trajSz=%zu\n",
                env->curr_timestep, loc, local_priority[i], (int)has_goal, nb,
                trajLNS_.trajs[i].size());
        }

        // Iron lane: redirect counterflow agents in lane to exit
        // (runs BEFORE require_guide_path check so A* computes to exit)
        if (optsvr::cfg().iron_lane_enabled) {
            int loc = env->curr_states[i].location;
            if (loc >= 0 && IronLane::is_lane_cell(loc)) {
                int lock = IronLane::g_lane_lock_dir;
                if (lock != 0) {
                    int tid = env->curr_task_schedule[i];
                    int exit = -1;
                    if (tid >= 0) {
                        int goal_comp = 0;
                        auto it = env->task_pool.find(tid);
                        if (it != env->task_pool.end() && !it->second.locations.empty()) {
                            int idx = it->second.idx_next_loc;
                            if (idx >= 0 && idx < (int)it->second.locations.size())
                                goal_comp = IronLane::get_component(it->second.locations[idx]);
                        }
                        if (goal_comp != 0) {
                            int dir = (goal_comp == 2) ? 1 : -1;
                            if (dir != lock) {
                                // Counterflow: redirect to opposite side
                                exit = (goal_comp == 2)
                                    ? (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1)
                                    : (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1);
                            }
                        }
                    } else {
                        // Idle in lane: redirect per lock direction
                        exit = (lock > 0)
                            ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                            : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
                    }
                    if (exit >= 0) {
                        trajLNS_.tasks[i] = exit;
                    } else if (tid >= 0) {
                        int rc_goal_comp = 0;
                        auto rit = env->task_pool.find(tid);
                        if (rit != env->task_pool.end() && !rit->second.locations.empty()) {
                            int ridx = rit->second.idx_next_loc;
                            if (ridx >= 0 && ridx < (int)rit->second.locations.size())
                                rc_goal_comp = IronLane::get_component(rit->second.locations[ridx]);
                        }
                        int rdir = (rc_goal_comp == 2) ? 1 : (rc_goal_comp == 1) ? -1 : 0;
                        if (optsvr::cfg().iron_lane_debug)
                            std::fprintf(stderr, "[LaneFail] a=%d,loc=%d,lock=%+d,gc=%d,dir=%+d,exit=%d\\n",
                            i, loc, lock, rc_goal_comp, rdir, exit);
                    }
                } else {
                    // Agent in lane cell but lock==0 — dump all
                    int tid = env->curr_task_schedule[i];
                    int gc2 = 0, dir2 = 0;
                    if (tid >= 0) {
                        auto it2 = env->task_pool.find(tid);
                        if (it2 != env->task_pool.end() && !it2->second.locations.empty()) {
                            int idx2 = it2->second.idx_next_loc;
                            if (idx2 >= 0 && idx2 < (int)it2->second.locations.size())
                                gc2 = IronLane::get_component(it2->second.locations[idx2]);
                            dir2 = (gc2 == 2) ? 1 : (gc2 == 1) ? -1 : 0;
                        }
                    }
                    if (optsvr::cfg().iron_lane_debug)
                        std::fprintf(stderr, "[InLane] a=%d,loc=%d,lock=%+d,gc=%d,dir=%+d,tid=%d,task=%d\\n",
                        i, loc, 0, gc2, dir2, tid, env->curr_task_schedule[i]);
                }
            }
        }

        // 检测 guide-path mismatch / need replan
        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty()) {
            require_guide_path_[i] = true;
        } else if (trajLNS_.trajs[i].back() != trajLNS_.tasks[i]) {
            require_guide_path_[i] = true;
        }

        // 更新 per-agent 状态
        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (decided_[i].loc == -1) {
            decided_[i].loc = env->curr_states[i].location;
            assert(decided_[i].state == DONE::DONE);
        }
        if (prev_states_[i].location == decided_[i].loc) {
            decided_[i].state = DONE::DONE;
        }
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        // 跨 episode 的优先级更新
        if (require_guide_path_[i])
            local_priority[i] = p_copy_[i];
        else if (!env->goal_locations[i].empty())
            local_priority[i] = local_priority[i] + 1;

        if (!env->goal_locations[i].empty() &&
            trajLNS_.neighbors[env->curr_states[i].location].size() == 1) {
            local_priority[i] = local_priority[i] + 10;
        }
    }
}

// ============ update_guide_paths_once_for_multistep ============
void OptPlanner::update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
{
    const auto guide_start = std::chrono::steady_clock::now();
    pressure_stats_.guide_replanned = 0;
    pressure_stats_.fw_replanned = 0;

    // Determine extra_weights pointer: only pass if WARE_DIRECT is enabled and weights are built
    const std::vector<std::vector<int>>* ew_ptr =
        (!extra_weights_.empty()) ? &extra_weights_ : nullptr;

    // Guide path order: sequential by default, LRU if configured
    std::vector<int> need_ids;
    need_ids.reserve(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
        if (require_guide_path_[i])
            need_ids.push_back(i);
    }
    if (optsvr::cfg().iron_lane_lru_sort) {
        // LRU排序：等最久的先算
        std::sort(need_ids.begin(), need_ids.end(), [this](int a, int b) {
            return last_guide_tick_[a] < last_guide_tick_[b];
        });
    }
    // else: sequential order (as collected from 0 to num_of_agents)

    for (int idx : need_ids) {
        if (std::chrono::steady_clock::now() > flow_end_time)
            break;
        if (!trajLNS_.trajs[idx].empty())
            remove_traj(trajLNS_, idx);
        if (update_traj(trajLNS_, idx, nullptr, ew_ptr)) {
            pressure_stats_.guide_replanned++;
            last_guide_tick_[idx] = pressure_stats_.tick;
        }
    }
    const auto fw_start = std::chrono::steady_clock::now();
    pressure_stats_.guide_ms =
        std::chrono::duration<double, std::milli>(fw_start - guide_start).count();

    // 串行 Frank-Wolfe
    std::unordered_set<int> updated;
    frank_wolfe(trajLNS_, updated, flow_end_time, ew_ptr);
    const auto fw_end = std::chrono::steady_clock::now();
    pressure_stats_.fw_replanned = static_cast<int>(updated.size());
    pressure_stats_.fw_ms =
        std::chrono::duration<double, std::milli>(fw_end - fw_start).count();
}

void OptPlanner::log_plan_pressure_if_needed(SharedEnvironment* env) const
{
    if (env == nullptr || map_profile().type != MapType::IRON_HARVEST_DOMAIN) {
        return;
    }

    auto pct = [](int num, int den) -> double {
        return den > 0 ? 100.0 * static_cast<double>(num) / static_cast<double>(den) : 0.0;
    };
    const double avg_h_delta = pressure_stats_.h_samples > 0
        ? static_cast<double>(pressure_stats_.h_delta_sum) /
              static_cast<double>(pressure_stats_.h_samples)
        : 0.0;
    const double avg_max_cell = pressure_stats_.density_steps > 0
        ? static_cast<double>(pressure_stats_.density_max_cell_sum) /
              static_cast<double>(pressure_stats_.density_steps)
        : 0.0;
    const double avg_hot_cells = pressure_stats_.density_steps > 0
        ? static_cast<double>(pressure_stats_.hot_cell_samples) /
              static_cast<double>(pressure_stats_.density_steps)
        : 0.0;
    const double avg_hot_agents = pressure_stats_.density_steps > 0
        ? static_cast<double>(pressure_stats_.hot_agent_samples) /
              static_cast<double>(pressure_stats_.density_steps)
        : 0.0;
    const double avg_busy = pressure_stats_.density_steps > 0
        ? static_cast<double>(pressure_stats_.busy_samples) /
              static_cast<double>(pressure_stats_.density_steps)
        : 0.0;
    const double avg_active = pressure_stats_.density_steps > 0
        ? static_cast<double>(pressure_stats_.active_samples) /
              static_cast<double>(pressure_stats_.density_steps)
        : 0.0;

    std::ostringstream os;
    os << std::fixed << std::setprecision(1);
    os << "[OptPlanner] tick=" << pressure_stats_.tick
       << " replan(need=" << pressure_stats_.guide_required
       << ",guide=" << pressure_stats_.guide_replanned
       << ",fw=" << pressure_stats_.fw_replanned
       << ",total=" << (pressure_stats_.guide_replanned + pressure_stats_.fw_replanned)
       << ") steps=" << pressure_stats_.produced_steps << "/" << pressure_stats_.requested_steps
       << " budget(flow=" << pressure_stats_.flow_budget_ms
       << "ms,pibt=" << pressure_stats_.pibt_reserved_ms
       << "ms,margin=" << pressure_stats_.deadline_margin_ms
       << "ms,recent=" << pressure_stats_.recent_pibt_avg_ms
       << "/" << pressure_stats_.recent_pibt_max_ms << "ms)"
       << " time(setup=" << pressure_stats_.setup_ms
       << "ms,guide=" << pressure_stats_.guide_ms
       << "ms,fw=" << pressure_stats_.fw_ms
       << "ms,pibt=" << pressure_stats_.pibt_ms
       << "ms,total=" << pressure_stats_.total_ms << "ms)"
       << " cong(avgBusy=" << avg_busy
       << ",avgActive=" << avg_active
       << ",fw=" << pct(pressure_stats_.fw_samples, pressure_stats_.busy_samples) << "%"
       << ",turn=" << pct(pressure_stats_.turn_samples, pressure_stats_.busy_samples) << "%"
       << ",wait=" << pct(pressure_stats_.wait_samples, pressure_stats_.busy_samples) << "%"
       << ",actW=" << pct(pressure_stats_.active_wait_samples, pressure_stats_.active_samples) << "%"
       << ",prog=" << pct(pressure_stats_.progress_samples, pressure_stats_.h_samples) << "%"
       << ",noProg=" << pct(pressure_stats_.no_progress_samples, pressure_stats_.h_samples) << "%"
       << ",reg=" << pct(pressure_stats_.regress_samples, pressure_stats_.h_samples) << "%"
       << ",dh=" << avg_h_delta
       << ",hot=max" << avg_max_cell << "/" << pressure_stats_.density_max_cell_peak
       << ",cells=" << avg_hot_cells
       << ",load=" << avg_hot_agents
       << ",wait=" << pct(pressure_stats_.hot_wait_samples,
                           pressure_stats_.hot_agent_samples) << "%)";

    // CSV log: guide path gap + time breakdown
    static FILE* csv = nullptr;
    if (!csv) {
        csv = std::fopen("planner_debug.csv", "w");
        if (csv) std::fprintf(csv, "tick,need,guide,gap,pibt_ms,guide_ms,total_ms\n");
    }
    if (csv) {
        int gap = pressure_stats_.guide_required - pressure_stats_.guide_replanned;
        std::fprintf(csv, "%d,%d,%d,%d,%.1f,%.1f,%.1f\n",
            pressure_stats_.tick,
            pressure_stats_.guide_required,
            pressure_stats_.guide_replanned,
            gap > 0 ? gap : 0,
            pressure_stats_.pibt_ms,
            pressure_stats_.guide_ms,
            pressure_stats_.total_ms);
        std::fflush(csv);
    }

    std::cerr << os.str() << "\n";
}

double OptPlanner::recent_pibt_avg_ms() const
{
    if (recent_pibt_count_ <= 0) return 0.0;
    const int n = std::min(recent_pibt_count_, static_cast<int>(recent_pibt_ms_.size()));
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += recent_pibt_ms_[i];
    }
    return sum / static_cast<double>(n);
}

double OptPlanner::recent_pibt_max_ms() const
{
    if (recent_pibt_count_ <= 0) return 0.0;
    const int n = std::min(recent_pibt_count_, static_cast<int>(recent_pibt_ms_.size()));
    double mx = 0.0;
    for (int i = 0; i < n; i++) {
        mx = std::max(mx, recent_pibt_ms_[i]);
    }
    return mx;
}

void OptPlanner::record_pibt_ms(double pibt_ms)
{
    recent_pibt_ms_[recent_pibt_next_] = pibt_ms;
    recent_pibt_next_ = (recent_pibt_next_ + 1) % static_cast<int>(recent_pibt_ms_.size());
    recent_pibt_count_ = std::min(recent_pibt_count_ + 1,
                                  static_cast<int>(recent_pibt_ms_.size()));
}

// ============ refresh_multistep_step_state ============
void OptPlanner::refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            // 空闲 agent: 目标=当前位置，极低优先级
            trajLNS_.tasks[i] = env->curr_states[i].location;
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
            local_priority[i] = local_priority[i] + 1;
        }

        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (prev_states_[i].location == decided_[i].loc) {
            decided_[i].state = DONE::DONE;
        }
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        if (!env->goal_locations[i].empty() &&
            trajLNS_.neighbors[env->curr_states[i].location].size() == 1) {
            local_priority[i] = local_priority[i] + 10;
        }
    }
}

// ============ run_multistep_pibt_once ============
void OptPlanner::run_multistep_pibt_once(
    SharedEnvironment* env, std::vector<double>& local_priority,
    std::vector<Action>& one_step_actions)
{
    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
        return local_priority.at(a) > local_priority.at(b);
    });

    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == DONE::NOT_DONE) {
            continue;
        }
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env->map.size());
            causalPIBT(i, -1, prev_states_, next_states_,
                       prev_decision_, decision_,
                       occupied_, trajLNS_);
        }
    }

    one_step_actions.resize(env->num_of_agents);
    for (int id : ids_) {
        if (next_states_.at(id).location != -1)
            decision_.at(next_states_.at(id).location) = -1;
        if (next_states_.at(id).location >= 0) {
            decided_.at(id) = DCR({next_states_.at(id).location, DONE::NOT_DONE});
        }
        one_step_actions.at(id) = getAction(prev_states_.at(id), decided_.at(id).loc, env);
        checked_.at(id) = false;
    }

    for (int id = 0; id < env->num_of_agents; id++) {
        if (!checked_.at(id) && one_step_actions.at(id) == Action::FW) {
            moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
        }
    }

    prev_states_ = next_states_;
}

// ============ accumulate_congestion_stats ============
void OptPlanner::accumulate_congestion_stats(
    SharedEnvironment* env,
    const std::vector<Action>& one_step_actions)
{
    if (env == nullptr || env->num_of_agents <= 0) {
        return;
    }
    const int n_agents = std::min<int>(
        env->num_of_agents,
        std::min<int>(env->curr_states.size(), one_step_actions.size()));
    if (n_agents <= 0) {
        return;
    }

    constexpr int GRID_CELL = 8;
    const int grid_rows = std::max(1, (env->rows + GRID_CELL - 1) / GRID_CELL);
    const int grid_cols = std::max(1, (env->cols + GRID_CELL - 1) / GRID_CELL);
    const int grid_count = grid_rows * grid_cols;
    std::vector<int> grid_busy(grid_count, 0);
    std::vector<int> grid_wait(grid_count, 0);

    int step_busy = 0;
    for (int aid = 0; aid < n_agents; aid++) {
        const bool busy = aid < static_cast<int>(env->goal_locations.size()) &&
                          !env->goal_locations[aid].empty();
        if (!busy) {
            continue;
        }

        const State& curr = env->curr_states[aid];
        if (curr.location < 0 || curr.location >= static_cast<int>(env->map.size())) {
            continue;
        }
        const Action action = one_step_actions[aid];
        const int goal = env->goal_locations[aid].front().first;
        const bool active = curr.location != goal;

        pressure_stats_.busy_samples++;
        step_busy++;
        if (active) {
            pressure_stats_.active_samples++;
        }

        if (action == Action::FW) {
            pressure_stats_.fw_samples++;
        } else if (action == Action::CR || action == Action::CCR) {
            pressure_stats_.turn_samples++;
        } else if (action == Action::W) {
            pressure_stats_.wait_samples++;
            if (active) {
                pressure_stats_.active_wait_samples++;
            }
        }

        if (active) {
            const int h_before = get_h_fast(env, curr.location, goal, curr.orientation);
            const State next = rollout_next_state(curr, action, env);
            const int h_after = get_h_fast(env, next.location, goal, next.orientation);
            pressure_stats_.h_samples++;
            pressure_stats_.h_delta_sum += static_cast<long long>(h_before - h_after);
            if (h_after < h_before) {
                pressure_stats_.progress_samples++;
            } else if (h_after > h_before) {
                pressure_stats_.regress_samples++;
            } else {
                pressure_stats_.no_progress_samples++;
            }
        }

        const int row = curr.location / env->cols;
        const int col = curr.location % env->cols;
        const int gr = std::min(grid_rows - 1, std::max(0, row / GRID_CELL));
        const int gc = std::min(grid_cols - 1, std::max(0, col / GRID_CELL));
        const int gid = gr * grid_cols + gc;
        grid_busy[gid]++;
        if (action == Action::W) {
            grid_wait[gid]++;
        }
    }

    pressure_stats_.action_samples += n_agents;
    pressure_stats_.density_steps++;

    int max_cell = 0;
    for (int load : grid_busy) {
        max_cell = std::max(max_cell, load);
    }
    pressure_stats_.density_max_cell_sum += max_cell;
    pressure_stats_.density_max_cell_peak =
        std::max(pressure_stats_.density_max_cell_peak, max_cell);

    const int avg_busy_per_cell = grid_count > 0
        ? (step_busy + grid_count - 1) / grid_count
        : step_busy;
    const int hot_threshold = std::max(12, avg_busy_per_cell * 3);
    for (int gid = 0; gid < grid_count; gid++) {
        if (grid_busy[gid] >= hot_threshold) {
            pressure_stats_.hot_cell_samples++;
            pressure_stats_.hot_agent_samples += grid_busy[gid];
            pressure_stats_.hot_wait_samples += grid_wait[gid];
        }
    }
}

// ============ append_actions_and_rollout_states ============
void OptPlanner::append_actions_and_rollout_states(
    SharedEnvironment* env,
    std::vector<std::vector<Action>>& actions,
    const std::vector<Action>& one_step_actions)
{
    for (int aid = 0; aid < env->num_of_agents; aid++) {
        actions[aid].push_back(one_step_actions[aid]);
    }

    std::vector<State> rolled_states(env->num_of_agents);
    for (int aid = 0; aid < env->num_of_agents; aid++) {
        rolled_states[aid] = rollout_next_state(env->curr_states[aid], one_step_actions[aid], env);
    }
    env->curr_states = rolled_states;
    env->curr_timestep += 1;
}

// ============ plan ============
void OptPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                      SharedEnvironment* env, int num_steps)
{
    actions.clear();
    if (env == nullptr || env->num_of_agents <= 0) {
        return;
    }

    actions.resize(env->num_of_agents);
    if (num_steps <= 0) {
        return;
    }

    const auto episode_start = std::chrono::steady_clock::now();
    pressure_stats_ = PlanPressureStats();
    pressure_stats_.tick = env->system_timestep;
    pressure_stats_.requested_steps = num_steps;

    // 时间预算分配先用保守基线。IRON 会在 setup 后根据 guide backlog
    // 和最近 PIBT 实测耗时动态调整，避免前期过早挤压 guide path 计算。
    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    int pibt_reserved_ms = pibt_time * num_steps;
    int deadline_margin_ms = 5;

    const bool iron_map = (map_profile().type == MapType::IRON_HARVEST_DOMAIN);

    std::vector<double> local_priority = p_;

    const std::vector<State> original_states = env->curr_states;
    const int original_timestep = env->curr_timestep;
    const std::vector<DCR> original_decided = decided_;

    decided_.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

    // --- One-time setup ---
    initialize_dummy_goals_if_needed(env);

    const auto setup_start = std::chrono::steady_clock::now();
    setup_multistep_episode_state(env, episode_start, local_priority);
    const auto setup_end = std::chrono::steady_clock::now();
    pressure_stats_.setup_ms =
        std::chrono::duration<double, std::milli>(setup_end - setup_start).count();
    pressure_stats_.guide_required =
        static_cast<int>(std::count(require_guide_path_.begin(), require_guide_path_.end(), true));

    if (iron_map) {
        const int base_reserve = pibt_time * num_steps;
        const double guide_pressure = env->num_of_agents > 0
            ? static_cast<double>(pressure_stats_.guide_required) /
                  static_cast<double>(env->num_of_agents)
            : 0.0;
        const double recent_avg = recent_pibt_avg_ms();
        const double recent_max = recent_pibt_max_ms();
        pressure_stats_.recent_pibt_avg_ms = recent_avg;
        pressure_stats_.recent_pibt_max_ms = recent_max;
        const bool has_recent_pibt = recent_pibt_count_ >= 2;
        const int recent_reserve = has_recent_pibt
            ? static_cast<int>(std::ceil(std::max(recent_avg * 1.25, recent_max * 1.10)))
            : 0;

        deadline_margin_ms = std::max(30, time_limit / 50);  // >=2%

        if (guide_pressure >= 0.20) {
            // 前期大量 agent 缺 guide path，把更多时间留给 guide。
            pibt_reserved_ms = base_reserve + std::max(40, time_limit / 40);  // >=2.5%
            if (has_recent_pibt) {
                const int measured = recent_reserve + std::max(20, time_limit / 100);
                const int min_reserve = std::max(pibt_time, base_reserve * 2 / 3);
                pibt_reserved_ms = std::max(min_reserve, std::min(pibt_reserved_ms, measured));
            }
        } else if (guide_pressure >= 0.02) {
            // backlog 已下降但仍明显，轻微抬高 PIBT 余量。
            pibt_reserved_ms = base_reserve + std::max(70, time_limit / 30);
            if (has_recent_pibt) {
                const int measured = recent_reserve + std::max(25, time_limit / 80);
                const int min_reserve = std::max(pibt_time, base_reserve * 3 / 4);
                pibt_reserved_ms = std::max(min_reserve, std::min(pibt_reserved_ms, measured));
            }
        } else {
            // 后期 guide 压力很低，FW/PIBT 抖动成为主要 timeout 来源。
            pibt_reserved_ms = (base_reserve * 5 + 3) / 4 + std::max(80, time_limit / 20);
            if (has_recent_pibt) {
                const int measured = recent_reserve + std::max(40, time_limit / 50);
                const int min_reserve = std::max(pibt_time, base_reserve * 3 / 4);
                pibt_reserved_ms = std::max(min_reserve, std::min(pibt_reserved_ms, measured));
            }
        }

        if (!has_recent_pibt && guide_pressure < 0.20 &&
            (pibt_ms_ema_ > 0.0 || pibt_ms_peak_ > 0.0)) {
            const int adaptive_reserve = static_cast<int>(std::ceil(
                std::max(pibt_ms_ema_ * 1.25, pibt_ms_peak_ * 1.10)));
            pibt_reserved_ms = std::max(pibt_reserved_ms, adaptive_reserve);
        }

        const int max_reserve = guide_pressure >= 0.20
            ? std::max(base_reserve, time_limit * 25 / 100)
            : std::max(base_reserve, time_limit * 45 / 100);
        pibt_reserved_ms = std::min(pibt_reserved_ms, max_reserve);
    }

    const int flow_budget_ms = std::max(
        0, time_limit - pibt_reserved_ms - deadline_margin_ms -
               TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    pressure_stats_.flow_budget_ms = flow_budget_ms;
    pressure_stats_.pibt_reserved_ms = pibt_reserved_ms;
    pressure_stats_.deadline_margin_ms = deadline_margin_ms;

    TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

    // 硬截止时间：time_limit - 安全余量（确保无论如何都能在截止前返回）
    TimePoint hard_deadline = episode_start + std::chrono::milliseconds(
        std::max(0, time_limit - deadline_margin_ms));

    update_guide_paths_once_for_multistep(env, flow_end_time);

    // Guide path gap diagnostic
    if (iron_map && optsvr::cfg().iron_lane_debug) {
        int still_need = 0;
        for (int i = 0; i < env->num_of_agents; i++)
            if (require_guide_path_[i]) still_need++;
        static int g_last_gap_tick = -1;
        int cur = env->curr_timestep;
        if (cur >= 0 && cur != g_last_gap_tick) {
            g_last_gap_tick = cur;
            std::fprintf(stderr, "[GuideGap] tick=%d need=%d guide=%d gap=%d remain=%d total=%d gapPct=%.1f%%\n",
                cur,
                pressure_stats_.guide_required,
                pressure_stats_.guide_replanned,
                pressure_stats_.guide_required - pressure_stats_.guide_replanned,
                still_need,
                env->num_of_agents,
                pressure_stats_.guide_required > 0
                    ? 100.0 * (pressure_stats_.guide_required - pressure_stats_.guide_replanned)
                        / pressure_stats_.guide_required
                    : 0.0);
        }
    }

    // commit cross-episode priority
    p_ = local_priority;

    // Iron lane traffic light: ORZ-style vote (every VOTE_PERIOD ticks)
    if (optsvr::cfg().iron_lane_enabled) {
        int ts = (env->system_timestep >= 0) ? env->system_timestep : env->curr_timestep;
        static int g_next_vote = 0;
        int period = std::max(1, optsvr::cfg().iron_lane_vote_period);
        int guard_dist = std::max(1, optsvr::cfg().iron_lane_guard_dist);
        if (ts >= g_next_vote) {
            g_next_vote = ts + period;

            // Step 1: Count agents for vote
            // Two sources: in-lane agents and approaching agents (within guard_dist)
            int core_cnt = 0, in_12 = 0, in_21 = 0, app_12 = 0, app_21 = 0;
            for (int a = 0; a < env->num_of_agents; a++) {
                int tid = env->curr_task_schedule[a];
                if (tid < 0) continue;
                int loc = env->curr_states[a].location;
                if (loc < 0) continue;
                // Core occupancy (for freeze/clearance check)
                if (IronLane::is_core_cell(loc)) core_cnt++;
                // Get task goal
                auto it = env->task_pool.find(tid);
                if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                int idx = it->second.idx_next_loc;
                if (idx < 0 || idx >= (int)it->second.locations.size()) continue;
                int goal = it->second.locations[idx];
                int goal_comp = IronLane::get_component(goal);
                if (goal_comp != 1 && goal_comp != 2) continue;  // only crossing counts
                int dir = (goal_comp == 2) ? 1 : -1;  // +1 = toward comp2, -1 = toward comp1

                if (IronLane::is_lane_cell(loc)) {
                    // Agent already in the lane — count by direction
                    if (dir > 0) in_12++;
                    else in_21++;
                } else {
                    // Agent outside the lane — check if approaching to cross
                    int agent_comp = IronLane::get_component(loc);
                    if (agent_comp == goal_comp) continue;  // same side, no crossing
                    int x = loc % env->cols, y = loc / env->cols;
                    int d = IronLane::manhattan_to_lane(x, y);
                    if (d > guard_dist) continue;
                    if (dir > 0) app_12++;
                    else app_21++;
                }
            }

            // Also count blocked/passed for stats (not used in decision)
            int blk12 = IronLane::g_blocked_12;
            int blk21 = IronLane::g_blocked_21;
            int pass12 = IronLane::g_passed_12;
            int pass21 = IronLane::g_passed_21;

            // Total demand: in-lane + approaching
            int total_12 = in_12 + app_12;
            int total_21 = in_21 + app_21;

            int prev_lock = IronLane::g_lane_lock_dir;
            int new_lock = prev_lock;

            // ── Step 2: Clearance handling ────────────────────────────────
            // Three mutually-exclusive cases:
            //   A) Already in clearance → check if core empty to exit
            //   B) Normal vote → may enter clearance on flip
            //   C) Initial first-vote sweep (agents placed in lane by scenario)

            static bool g_initial_sweep_done = false;

            if (IronLane::g_clearance_mode) {
                // ── Case A: In clearance → check completion ──────────────
                if (core_cnt == 0) {
                    // Core empty → exit clearance, apply pending direction
                    int to_dir = IronLane::g_clearance_to_dir;
                    new_lock = to_dir;

                    // Clear redirect list
                    IronLane::g_redirected_count = 0;

                    // Sweep lane: redirect counterflow agents + idle agents
                    for (int a = 0; a < env->num_of_agents; a++) {
                        int loc = env->curr_states[a].location;
                        if (loc < 0) continue;
                        if (!IronLane::is_lane_cell(loc)) continue;
                        int tid = env->curr_task_schedule[a];
                        int exit = -1;
                        if (tid >= 0) {
                            auto it = env->task_pool.find(tid);
                            if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                            int idx = it->second.idx_next_loc;
                            if (idx < 0 || idx >= (int)it->second.locations.size()) continue;
                            int goal = it->second.locations[idx];
                            int goal_comp = IronLane::get_component(goal);
                            int dir = 0;
                            if (goal_comp == 2) dir = 1;
                            else if (goal_comp == 1) dir = -1;
                            if (dir == 0 || dir == to_dir) continue;  // not counterflow
                            // Counterflow: retreat to opposite side
                            exit = (goal_comp == 2)
                                ? (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1)
                                : (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1);
                        } else {
                            // Idle: redirect to new direction side
                            exit = (to_dir > 0)
                                ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                                : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
                        }
                        if (exit >= 0 && IronLane::g_redirected_count < IronLane::MAX_AGENTS) {
                            IronLane::g_redirected_agents[IronLane::g_redirected_count++] = a;
                            if (optsvr::cfg().iron_lane_debug)
                                std::fprintf(stderr, "[IronLane] clearance_redirect a=%d loc=%d to_dir=%+d exit=%d\n",
                                a, loc, to_dir, exit);
                        }
                    }

                    if (optsvr::cfg().iron_lane_debug)
                        std::fprintf(stderr, "[IronLane] clearance_end tick=%d core=%d dir=%+d redirects=%d\n",
                        ts, core_cnt, to_dir, IronLane::g_redirected_count);

                    // Reset clearance state
                    IronLane::g_clearance_mode = false;
                    IronLane::g_clearance_start_tick = -1;
                    IronLane::g_clearance_to_dir = 0;
                    IronLane::g_clearance_trigger = 0;
                } else {
                    // Still clearing — keep lock=0 (block both sides)
                    new_lock = 0;
                }
            } else {
                // ── Case B/C: Not in clearance → normal vote ─────────────

                // Decision
                // Priority 1: Vote ratio ≥ threshold → flip
                int ratio_th = std::max(1, optsvr::cfg().iron_lane_flow_ratio_flip);
                if (total_12 >= total_21 * ratio_th) {
                    new_lock = 1;
                } else if (total_21 >= total_12 * ratio_th) {
                    new_lock = -1;
                }
                // Priority 2: Core > threshold → freeze
                else if (core_cnt > optsvr::cfg().iron_lane_core_flip_th) {
                    new_lock = prev_lock;
                }
                // Priority 3: Normal majority vote
                else if (total_12 > total_21) {
                    new_lock = 1;
                } else if (total_21 > total_12) {
                    new_lock = -1;
                }
                // else tie → keep prev_lock

                // ── Case C: First vote initial sweep ──────────────────────
                // (agents placed in lane by scenario may be going wrong way)
                if (!g_initial_sweep_done) {
                    g_initial_sweep_done = true;
                    // Redirect counterflow agents relative to current lock
                    // (same-as-flip redirect but without changing the lock)
                    IronLane::g_redirected_count = 0;
                    for (int a = 0; a < env->num_of_agents; a++) {
                        int loc = env->curr_states[a].location;
                        if (loc < 0) continue;
                        if (!IronLane::is_lane_cell(loc)) continue;
                        int tid = env->curr_task_schedule[a];
                        int exit = -1;
                        if (tid >= 0) {
                            auto it = env->task_pool.find(tid);
                            if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                            int idx = it->second.idx_next_loc;
                            if (idx < 0 || idx >= (int)it->second.locations.size()) continue;
                            int goal = it->second.locations[idx];
                            int goal_comp = IronLane::get_component(goal);
                            int dir = 0;
                            if (goal_comp == 2) dir = 1;
                            else if (goal_comp == 1) dir = -1;
                            if (dir == 0 || dir == new_lock) continue;  // not counterflow
                            exit = (goal_comp == 2)
                                ? (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1)
                                : (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1);
                        } else {
                            exit = (new_lock > 0)
                                ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                                : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
                        }
                        if (exit >= 0 && IronLane::g_redirected_count < IronLane::MAX_AGENTS) {
                            IronLane::g_redirected_agents[IronLane::g_redirected_count++] = a;
                            if (optsvr::cfg().iron_lane_debug)
                                std::fprintf(stderr, "[IronLane] init_sweep a=%d loc=%d lock=%+d exit=%d\n",
                                a, loc, new_lock, exit);
                        }
                    }
                }

                // ── If flip detected → enter clearance mode ──────────────
                // Instead of immediately redirecting, block both sides and
                // wait for core to clear before applying the new direction.
                if (new_lock != prev_lock) {
                    // Determine trigger type for diagnostics
                    int trigger = 0;  // 0=ratio
                    if (!(total_12 >= total_21 * ratio_th || total_21 >= total_12 * ratio_th)) {
                        trigger = (core_cnt > optsvr::cfg().iron_lane_core_flip_th) ? 1 : 2;  // 1=core overflow, 2=normal
                    }

                    IronLane::g_clearance_mode = true;
                    IronLane::g_clearance_start_tick = ts;
                    IronLane::g_clearance_to_dir = new_lock;
                    IronLane::g_clearance_trigger = trigger;

                    if (optsvr::cfg().iron_lane_debug)
                        std::fprintf(stderr, "[IronLane] clearance_start tick=%d core=%d from=%+d to=%+d trigger=%d in_12=%d in_21=%d\n",
                        ts, core_cnt, prev_lock, new_lock, trigger,
                        in_12, in_21);

                    // Set lock=0 to block both sides during clearance
                    new_lock = 0;
                }
            }

            // Update stored lock
            IronLane::g_lane_lock_dir = new_lock;
            IronLane::g_prev_lock = prev_lock;

            // Step 3: Apply redirect tasks for redirected agents (vote-time)
            for (int ri = 0; ri < IronLane::g_redirected_count; ri++) {
                int a = IronLane::g_redirected_agents[ri];
                int loc = env->curr_states[a].location;
                if (loc < 0) continue;
                if (IronLane::is_lane_cell(loc)) {
                    int tid = env->curr_task_schedule[a];
                    int exit = -1;
                    if (tid >= 0) {
                        int goal_comp = 0;
                        auto it = env->task_pool.find(tid);
                        if (it != env->task_pool.end() && !it->second.locations.empty()) {
                            int idx = it->second.idx_next_loc;
                            if (idx >= 0 && idx < (int)it->second.locations.size())
                                goal_comp = IronLane::get_component(it->second.locations[idx]);
                        }
                        exit = (goal_comp == 2)
                            ? (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1)
                            : (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1);
                    } else {
                        int lock = IronLane::g_lane_lock_dir;
                        exit = (lock > 0)
                            ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                            : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
                    }
                    if (exit >= 0) {
                        trajLNS_.tasks[a] = exit;
                    }
                }
            }

            // Log and reset counters
            if (optsvr::cfg().iron_lane_debug)
                std::fprintf(stderr, "[IronLane] vote tick=%d core=%d in_12=%d in_21=%d app_12=%d app_21=%d tot_12=%d tot_21=%d blk=%d/%d pass=%d/%d prev=%d lock=%d%s\n",
                env->system_timestep, core_cnt,
                in_12, in_21, app_12, app_21, total_12, total_21,
                blk12, blk21, pass12, pass21,
                prev_lock, new_lock,
                IronLane::g_clearance_mode ? " CLEAR" : "");
            IronLane::g_blocked_12 = 0;
            IronLane::g_blocked_21 = 0;
            IronLane::g_passed_12 = 0;
            IronLane::g_passed_21 = 0;
        } // end if(ts >= g_next_vote)

        // Between votes: check agents in lane
        if (!IronLane::g_clearance_mode) {
            // Normal mode: existing priority boost + counterflow redirect
            for (int a = 0; a < env->num_of_agents; a++) {
            int loc = env->curr_states[a].location;
            if (loc < 0) continue;
            if (!IronLane::is_lane_cell(loc)) continue;
            int tid = env->curr_task_schedule[a];
            int lock = IronLane::g_lane_lock_dir;
            if (lock == 0) continue;  // no flow direction enforced

            int dir = 0, exit = -1;
            if (tid >= 0) {
                int goal_comp = 0;
                auto it = env->task_pool.find(tid);
                if (it != env->task_pool.end() && !it->second.locations.empty()) {
                    int idx = it->second.idx_next_loc;
                    if (idx >= 0 && idx < (int)it->second.locations.size())
                        goal_comp = IronLane::get_component(it->second.locations[idx]);
                }
                if (goal_comp != 0) {
                    dir = (goal_comp == 2) ? 1 : -1;
                    if (dir == lock) {
                        // Correct flow: no redirect
                        continue;
                    }
                    // Counterflow: redirect to opposite side
                    exit = (goal_comp == 2)
                        ? (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1)
                        : (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1);
                }
            } else {
                // Idle in lane → vacate, follow lock direction
                exit = (lock > 0)
                    ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                    : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
            }
            if (exit >= 0) {
                trajLNS_.tasks[a] = exit;  // safety keep-alive
                // Keep traj_dists — guide path was computed to exit by setup()+update_guide_paths
            }
        } // end for(a) between-votes
        } else {
            // Clearance mode: redirect idle agents in lane to exit
            int to_dir = IronLane::g_clearance_to_dir;
            for (int a = 0; a < env->num_of_agents; a++) {
                int loc = env->curr_states[a].location;
                if (loc < 0) continue;
                if (!IronLane::is_lane_cell(loc)) continue;
                int tid = env->curr_task_schedule[a];
                if (tid >= 0) continue;  // has task, continue naturally
                // Idle in lane during clearance → redirect out
                int exit = (to_dir > 0)
                    ? (IronLane::g_exit_to_comp2 ? IronLane::g_exit_to_comp2[loc] : -1)
                    : (IronLane::g_exit_to_comp1 ? IronLane::g_exit_to_comp1[loc] : -1);
                if (exit >= 0) {
                    trajLNS_.tasks[a] = exit;
                }
            }
        }
        // Removed per-tick CSV (was for diagnostics, adds significant overhead)
    }

    // PIBT priority with direction boost (copy, doesn't pollute local_priority)
    std::vector<double> pibt_priority = local_priority;
    if (optsvr::cfg().iron_lane_enabled) {
        int lock = IronLane::g_lane_lock_dir;
        if (lock != 0 && !IronLane::g_clearance_mode) {
            constexpr double DIR_DELTA = 1000.0;
            int guard_dist = std::max(1, optsvr::cfg().iron_lane_guard_dist);
            for (int a = 0; a < env->num_of_agents; a++) {
                int loc = env->curr_states[a].location;
                if (loc < 0) continue;
                int tid = env->curr_task_schedule[a];
                if (tid < 0) continue;
                auto it = env->task_pool.find(tid);
                if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                int idx = it->second.idx_next_loc;
                if (idx < 0 || idx >= (int)it->second.locations.size()) continue;
                int goal = it->second.locations[idx];
                int goal_comp = IronLane::get_component(goal);
                if (goal_comp != 1 && goal_comp != 2) continue;
                int dir = (goal_comp == 2) ? 1 : -1;

                if (IronLane::is_lane_cell(loc)) {
                    // In lane: direction vs lock
                    if (dir == lock)
                        pibt_priority[a] += DIR_DELTA;
                    else
                        pibt_priority[a] -= DIR_DELTA;
                } else {
                    // Outside lane: only if approaching to cross
                    int agent_comp = IronLane::get_component(loc);
                    if (agent_comp == goal_comp) continue;  // same side, not crossing
                    int x = loc % env->cols, y = loc / env->cols;
                    int d = IronLane::manhattan_to_lane(x, y);
                    if (d > guard_dist) continue;
                    if (dir == lock)
                        pibt_priority[a] += DIR_DELTA;
                    else
                        pibt_priority[a] -= DIR_DELTA;
                }
            }
        }
    }

    const auto pibt_start = std::chrono::steady_clock::now();
    for (int step = 0; step < num_steps; step++) {
        // 超时保护：如果接近硬截止时间，提前终止 PIBT 循环
        // 至少完成 1 步以保证有合法动作输出
        if (step > 0 && std::chrono::steady_clock::now() >= hard_deadline) {
            break;
        }

        std::vector<Action> one_step_actions;

        if (step > 0) {
            refresh_multistep_step_state(env, local_priority);
        }

        run_multistep_pibt_once(env, local_priority, one_step_actions);
        if (iron_map) {
            accumulate_congestion_stats(env, one_step_actions);
        }
        append_actions_and_rollout_states(env, actions, one_step_actions);

        // 保存每步动作，最后一步的将用于下一轮密度统计更新
        last_step_actions_ = one_step_actions;
        last_step_states_ = env->curr_states;  // rollout 后的状态
        pressure_stats_.produced_steps = step + 1;
    }
    const auto pibt_end = std::chrono::steady_clock::now();
    pressure_stats_.pibt_ms =
        std::chrono::duration<double, std::milli>(pibt_end - pibt_start).count();
    pressure_stats_.total_ms =
        std::chrono::duration<double, std::milli>(pibt_end - episode_start).count();
    record_pibt_ms(pressure_stats_.pibt_ms);
    if (pibt_ms_ema_ <= 0.0) {
        pibt_ms_ema_ = pressure_stats_.pibt_ms;
    } else {
        pibt_ms_ema_ = pibt_ms_ema_ * 0.75 + pressure_stats_.pibt_ms * 0.25;
    }
    pibt_ms_peak_ = std::max(pressure_stats_.pibt_ms, pibt_ms_peak_ * 0.90);

    // 恢复 env state
    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;

    log_plan_pressure_if_needed(env);
}

// ============ build_row_column_highway (DHFULL 版本: 80%阈值 + 软惩罚) ============
void OptPlanner::build_row_column_highway(SharedEnvironment* env)
{
    if (env == nullptr || env->rows <= 2 || env->cols <= 2) {
        return;
    }

    static constexpr int ROW_PENALTY = 250;
    static constexpr int COL_PENALTY = 150;

    // Initialize extra_weights: [map_size][4 directions], all zeros
    extra_weights_.assign(env->rows * env->cols, std::vector<int>(4, 0));

    // Build grid representation
    std::vector<std::vector<int>> grid(env->rows, std::vector<int>(env->cols, 1));
    for (int row = 0; row < env->rows; row++) {
        for (int col = 0; col < env->cols; col++) {
            grid[row][col] = env->map[row * env->cols + col];
        }
    }

    // --- Detect passage rows: 80% threshold + neighbor contrast ---
    std::vector<int> row_free_count(env->rows, 0);
    for (int i = 0; i < env->rows; i++) {
        for (int j = 0; j < env->cols; j++) {
            if (grid[i][j] == 0) row_free_count[i]++;
        }
    }

    int max_row_free = *std::max_element(row_free_count.begin(), row_free_count.end());
    int row_threshold = max_row_free * 80 / 100;

    std::vector<int> passage_rows;
    for (int i = 1; i < env->rows - 1; i++) {
        if (row_free_count[i] > row_threshold &&
            row_free_count[i - 1] <= row_threshold &&
            row_free_count[i + 1] <= row_threshold) {
            passage_rows.emplace_back(i);
        }
    }

    // Apply soft direction penalties to passage rows (alternating east/west)
    for (int i = 0; i < static_cast<int>(passage_rows.size()); i++) {
        int row = passage_rows[i];
        for (int col = 1; col < env->cols - 1; col++) {
            int loc = row * env->cols + col;
            if (i % 2 == 0) {
                extra_weights_[loc][2] = ROW_PENALTY;  // discourage west → flow east
            } else {
                extra_weights_[loc][0] = ROW_PENALTY;  // discourage east → flow west
            }
        }
    }

    // --- Detect passage columns: 80% threshold + neighbor contrast ---
    std::vector<int> col_free_count(env->cols, 0);
    for (int j = 0; j < env->cols; j++) {
        for (int i = 0; i < env->rows; i++) {
            if (grid[i][j] == 0) col_free_count[j]++;
        }
    }

    int max_col_free = *std::max_element(col_free_count.begin(), col_free_count.end());
    int col_threshold = max_col_free * 80 / 100;

    std::vector<int> passage_columns;
    for (int i = 1; i < env->cols - 1; i++) {
        if (col_free_count[i] > col_threshold &&
            col_free_count[i - 1] <= col_threshold &&
            col_free_count[i + 1] <= col_threshold) {
            passage_columns.emplace_back(i);
        }
    }

    // Apply soft direction penalties to passage columns (alternating south/north)
    for (int i = 0; i < static_cast<int>(passage_columns.size()); i++) {
        int col = passage_columns[i];
        for (int row = 1; row < env->rows - 1; row++) {
            int loc = row * env->cols + col;
            if (i % 2 == 0) {
                extra_weights_[loc][3] = COL_PENALTY;  // discourage north → flow south
            } else {
                extra_weights_[loc][1] = COL_PENALTY;  // discourage south → flow north
            }
        }
    }

    if (cfg().debug_level >= 1) {
        std::cerr << "[OptPlanner] WARE_DIRECT highway (DHFULL): passage_rows="
                  << passage_rows.size() << ", passage_columns="
                  << passage_columns.size()
                  << ", row_penalty=" << ROW_PENALTY
                  << ", col_penalty=" << COL_PENALTY << "\n";
    }
}

} // namespace optsvr
