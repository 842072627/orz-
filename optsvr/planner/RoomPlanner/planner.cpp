// optplan/planner.cpp — 优化版 Planner 实现
//
// 完整复制 DefaultPlanner::plan 的逻辑骨架，
// 维护自己的完整状态（TrajLNS、decision、priority 等）。

#include "planner.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "MapProfile.h"

// RoomPlan 命名空间下的副本（从 default_planner 复制，可自由修改）
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

using namespace RoomPlan;

namespace optsvr {

// ============ rollout_next_state ============
State RoomPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
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
void RoomPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
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

    // 初始化planner层stops计数器 (late_force_replan使用)
    planner_stops_.resize(env->num_of_agents, 0);

    // 分配初始优先级
    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < (int)ids_.size(); i++) {
        p_[ids_[i]] = ((double)(ids_.size() - i)) / ((double)(ids_.size() + 1));
    }
    p_copy_ = p_;
    initialized_ = true;

    // 初始化 PIBT 密度惩罚系统（R23: density_weight可通过环境变量启用）
    pibt_density_.initialize(env);

    // Warehouse 通道定向策略
    if (cfg().ware_direct && map_profile().type == MapType::WAREHOUSE_DOMAIN) {
        build_row_column_highway(env);
    }
}

// ============ initialize_dummy_goals_if_needed ============
void RoomPlanner::initialize_dummy_goals_if_needed(SharedEnvironment* env)
{
    if (env->curr_timestep != 0)
        return;
    dummy_goals_.resize(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
        dummy_goals_[i] = env->curr_states[i].location;
    }
}

// ============ setup_multistep_episode_state ============
void RoomPlanner::setup_multistep_episode_state(
    SharedEnvironment* env, TimePoint flow_end_time,
    std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        // 设置 task
        if (env->goal_locations[i].empty()) {
            // 空闲 agent: 极低优先级原地等待
            trajLNS_.tasks[i] = env->curr_states[i].location;
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        // 检测 guide-path mismatch / need replan
        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty()) {
            require_guide_path_[i] = true;
        } else if (trajLNS_.trajs[i].back() != trajLNS_.tasks[i]) {
            require_guide_path_[i] = true;
        }

        // late_force_replan — agent连续停留则强制重新规划guide path
        if (cfg().opt_room_late_force_replan > 0 &&
            env->curr_timestep > cfg().opt_room_replan_start_tick &&
            !env->goal_locations[i].empty()) {
            if (prev_states_[i].location == env->curr_states[i].location) {
                planner_stops_[i]++;
            } else {
                planner_stops_[i] = 0;
            }
            int replan_thresh = cfg().opt_room_late_force_replan;
            if (planner_stops_[i] >= replan_thresh) {
                require_guide_path_[i] = true;
                planner_stops_[i] = 0;
            }
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
void RoomPlanner::update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
{
    // Determine extra_weights pointer: only pass if WARE_DIRECT is enabled and weights are built
    const std::vector<std::vector<int>>* ew_ptr =
        (!extra_weights_.empty()) ? &extra_weights_ : nullptr;

    // 串行 guide path（与 DefaultPlanner 完全一致）
    for (int i = 0; i < env->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end_time)
            break;
        if (require_guide_path_[i]) {
            if (!trajLNS_.trajs[i].empty())
                remove_traj(trajLNS_, i);
            update_traj(trajLNS_, i, nullptr, ew_ptr);
        }
    }

    // 串行 Frank-Wolfe
    std::unordered_set<int> updated;
    frank_wolfe(trajLNS_, updated, flow_end_time, ew_ptr);
}

// ============ refresh_multistep_step_state ============
void RoomPlanner::refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            // 空闲 agent: 极低优先级
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
void RoomPlanner::run_multistep_pibt_once(
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

// ============ append_actions_and_rollout_states ============
void RoomPlanner::append_actions_and_rollout_states(
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
void RoomPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
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

    // 时间预算分配：
    // - 给 PIBT 留出固定时间（实测 5000 agents 单步 PIBT < 50ms）
    // - 剩余全部给 flow 阶段（guide path + frank_wolfe）
    // 原始估算 PIBT_RUNTIME_PER_100_AGENTS=1ms 对 5000 agents 只预留 50ms × num_steps，
    // 但 DefaultPlanner 还保留了 TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE=10ms 的容差
    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    const int flow_budget_ms = std::max(0, time_limit - pibt_time * num_steps - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

    // 硬截止时间：time_limit - 安全余量（确保无论如何都能在截止前返回）
    TimePoint hard_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit - 5));

    std::vector<double> local_priority = p_;

    const std::vector<State> original_states = env->curr_states;
    const int original_timestep = env->curr_timestep;
    const std::vector<DCR> original_decided = decided_;

    decided_.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

    // --- One-time setup ---
    initialize_dummy_goals_if_needed(env);

    setup_multistep_episode_state(env, flow_end_time, local_priority);

    update_guide_paths_once_for_multistep(env, flow_end_time);

    // commit cross-episode priority
    p_ = local_priority;

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
        append_actions_and_rollout_states(env, actions, one_step_actions);

        // 保存每步动作，最后一步的将用于下一轮密度统计更新
        last_step_actions_ = one_step_actions;
        last_step_states_ = env->curr_states;  // rollout 后的状态
    }

    // 恢复 env state
    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;
}

// ============ build_row_column_highway (DHFULL 版本: 80%阈值 + 软惩罚) ============
void RoomPlanner::build_row_column_highway(SharedEnvironment* env)
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
        std::cerr << "[RoomPlanner] WARE_DIRECT highway (DHFULL): passage_rows="
                  << passage_rows.size() << ", passage_columns="
                  << passage_columns.size()
                  << ", row_penalty=" << ROW_PENALTY
                  << ", col_penalty=" << COL_PENALTY << "\n";
    }
}

} // namespace optsvr
