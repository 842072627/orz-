// optsvr/planner/GameOpt/GameOptPlanner.cpp — Game 算例定制规划器实现
// 全量包含 FlowDecay 逻辑，GameOpt 特有策略内联于 gameOptPIBT 中。
// 不再委托 OptPlanner，不再依赖 pibt.cpp 中的 planner_mode==2 判断。
#include "GameOptPlanner.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "MapProfile.h"

// FlowDecay 子模块（直接引用，不经过 OptPlanner）
#include "../FlowDecay/heuristics.h"
#include "../FlowDecay/flow.h"
#include "../FlowDecay/search.h"
#include "../FlowDecay/pibt.h"
#include "../FlowDecay/utils.h"
#include "../FlowDecay/const.h"

#include <algorithm>
#include <numeric>
#include <unordered_set>
#include <chrono>
#include <iostream>
#include <cstdio>

using namespace OptPlan;

namespace optsvr {

// ============ rollout_next_state ============
State GameOptPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
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
void GameOptPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    assert(env->num_of_agents != 0);

    // 确保 PibtLock 已初始化
    pibt_lock().reset();

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

    // 初始化全局启发式表
    init_heuristics(env);
    mt1_.seed(0);
    srand(0);

    // placement new 初始化 TrajLNS
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

    printf("[GameOptPlanner] initialized (standalone), pibt_lock active\n");
}

// ============ initialize_dummy_goals_if_needed ============
void GameOptPlanner::initialize_dummy_goals_if_needed(SharedEnvironment* env)
{
    if (env->curr_timestep != 0) return;
    dummy_goals_.resize(env->num_of_agents);
    for (int i = 0; i < env->num_of_agents; i++) {
        dummy_goals_[i] = env->curr_states[i].location;
    }
}

// ============ setup_multistep_episode_state ============
void GameOptPlanner::setup_multistep_episode_state(
    SharedEnvironment* env, TimePoint flow_end_time,
    std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            // 默认：目标=当前位置，优先级最低
            trajLNS_.tasks[i] = env->curr_states[i].location;
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty()) {
            require_guide_path_[i] = true;
        } else if (trajLNS_.trajs[i].back() != trajLNS_.tasks[i]) {
            require_guide_path_[i] = true;
        }

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
void GameOptPlanner::update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
{
    // GameOpt 不使用 extra_weights（Warehouse 通道定向策略）
    const std::vector<std::vector<int>>* ew_ptr = nullptr;

    for (int i = 0; i < env->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end_time) break;
        if (require_guide_path_[i]) {
            if (!trajLNS_.trajs[i].empty())
                remove_traj(trajLNS_, i);
            update_traj(trajLNS_, i, nullptr, ew_ptr);
        }
    }

    std::unordered_set<int> updated;
    frank_wolfe(trajLNS_, updated, flow_end_time, ew_ptr);
}

// ============ refresh_multistep_step_state ============
void GameOptPlanner::refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
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

// ============ gameOptPIBT — GameOpt 专属 PIBT ============
// 与 FlowDecay causalPIBT 逻辑一致，额外内置 agent_blocked 强制等待
bool GameOptPlanner::gameOptPIBT(
    int curr_id, int higher_id,
    std::vector<State>& prev_states,
    std::vector<State>& next_states,
    std::vector<int>& prev_decision,
    std::vector<int>& decision,
    std::vector<bool>& occupied,
    TrajLNS& lns)
{
    assert(next_states[curr_id].location == -1);
    int prev_loc = prev_states[curr_id].location;
    int prev_orientation = prev_states[curr_id].orientation;
    int next_locs[4] = {
        prev_loc + 1,
        prev_loc + lns.env->cols,
        prev_loc - 1,
        prev_loc - lns.env->cols
    };
    int orien_next_v = next_locs[prev_orientation];

    assert(prev_loc >= 0 && prev_loc < (int)lns.env->map.size());

    int target = lns.tasks.at(curr_id);

    // ★ GameOpt 特有策略：scheduler 标记 agent_blocked → 强制原地等待
    if (curr_id < PibtLock::MAX_AGENTS &&
        pibt_lock().agent_blocked[curr_id]) {
        next_states.at(curr_id) = State(prev_loc, -1, -1);
        decision.at(prev_loc) = curr_id;
        return false;
    }

    // ★ IDEA: gradual penalty - reduce priority but don't force wait
    // (handled in run_multistep_pibt_once via local_priority adjustment)

    if (higher_id == -1) {
        occupied[prev_loc] = true;
    }

    // 生成候选
    std::vector<int> neighbors;
    std::vector<PIBT_C> successors;
    getNeighborLocs(&(lns.neighbors), neighbors, prev_loc);

    for (auto& neighbor : neighbors) {
        assert(validateMove(prev_loc, neighbor, lns.env));
        int min_heuristic = get_gp_h(lns, curr_id, neighbor);
        successors.emplace_back(neighbor, min_heuristic, -1, rand());
    }

    int wait_heuristic = get_gp_h(lns, curr_id, prev_loc);
    successors.emplace_back(prev_loc, wait_heuristic, -1, rand());

    // 排序：heuristic → 前走偏好 → 随机
    std::sort(successors.begin(), successors.end(),
        [&](PIBT_C& a, PIBT_C& b) {
            if (a.heuristic == b.heuristic) {
                if (a.location == orien_next_v && b.location != orien_next_v)
                    return true;
                if (a.location != orien_next_v && b.location == orien_next_v)
                    return false;
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });

    // ★ IDEA: PIBT counter-flow penalty - penalize moves against zone direction lock
    if (cfg().game_pibt_counter_flow) {
        auto& plock = pibt_lock();
        int n24c = plock.zone_n24c;
        if (n24c > 0) {
            int prev_r = prev_loc / lns.env->cols;
            int prev_c = prev_loc % lns.env->cols;
            int prev_zone = (prev_r / 24) * n24c + (prev_c / 24);
            int prev_zone_dir = (prev_zone >= 0 && prev_zone < PibtLock::MAX_ZONES)
                                ? plock.zone_dir[prev_zone] : 0;
            if (prev_zone_dir != 0) {
                int counter_cost = cfg().game_counter_flow_cost;
                for (auto& s : successors) {
                    if (s.location == prev_loc) continue; // wait is ok
                    int s_c = s.location % lns.env->cols;
                    int move_dir = (s_c > prev_c) ? +1 : (s_c < prev_c) ? -1 : 0;
                    if (move_dir != 0 && move_dir != prev_zone_dir) {
                        s.heuristic += counter_cost;
                    }
                }
                // Re-sort after penalty
                std::sort(successors.begin(), successors.end(),
                    [&](PIBT_C& a, PIBT_C& b) {
                        if (a.heuristic == b.heuristic) {
                            if (a.location == orien_next_v && b.location != orien_next_v)
                                return true;
                            if (a.location != orien_next_v && b.location == orien_next_v)
                                return false;
                            return a.tie_breaker < b.tie_breaker;
                        }
                        return a.heuristic < b.heuristic;
                    });
            }
        }
    }

    // 遍历候选
    for (auto& next : successors) {
        if (occupied[next.location] && !(higher_id == -1 && prev_loc == next.location))
            continue;
        assert(validateMove(prev_loc, next.location, lns.env));

        if (next.location == -1) continue;
        if (decision[next.location] != -1) continue;
        if (higher_id != -1 && prev_decision[next.location] == higher_id) continue;

        next_states.at(curr_id) = State(next.location, -1, -1);
        decision.at(next.location) = curr_id;

        if (prev_decision.at(next.location) != -1 &&
            next_states.at(prev_decision.at(next.location)).location == -1) {
            int lower_id = prev_decision.at(next.location);
            // ★ 递归也用 gameOptPIBT
            if (!gameOptPIBT(lower_id, curr_id, prev_states, next_states,
                             prev_decision, decision, occupied, lns)) {
                continue;
            }
        }

        return true;
    }

    // 所有候选失败 → 强制原地等待
    next_states.at(curr_id) = State(prev_loc, -1, -1);
    decision.at(prev_loc) = curr_id;
    return false;
}

// ============ run_multistep_pibt_once ============
void GameOptPlanner::run_multistep_pibt_once(
    SharedEnvironment* env, std::vector<double>& local_priority,
    std::vector<Action>& one_step_actions)
{
    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
        return local_priority.at(a) > local_priority.at(b);
    });

    // ★ IDEA7: urgency priority - boost agents close to their goal
    if (cfg().game_urgency_priority) {
        int thresh = cfg().game_urgency_threshold;
        float weight = cfg().game_urgency_weight;
        for (int i : ids_) {
            if (env->goal_locations[i].empty()) continue;
            int goal_loc = env->goal_locations[i].front().first;
            int curr_loc = env->curr_states[i].location;
            if (curr_loc < 0 || goal_loc < 0) continue;
            // Manhattan distance approximation
            int gr = goal_loc / env->cols, gc = goal_loc % env->cols;
            int cr = curr_loc / env->cols, cc = curr_loc % env->cols;
            int dist = std::abs(gr - cr) + std::abs(gc - cc);
            if (dist <= thresh && dist > 0) {
                local_priority[i] += weight / (float)(dist + 1);
            }
        }
        // Re-sort after urgency boost
        std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
            return local_priority.at(a) > local_priority.at(b);
        });
    }

    // ★ Apply urgency priority boost from scheduler
    if (cfg().game_urgency_priority) {
        auto& plock = pibt_lock();
        for (int i : ids_) {
            if (i < PibtLock::MAX_AGENTS && plock.agent_priority_penalty[i] != 0.0f) {
                local_priority[i] += plock.agent_priority_penalty[i];
            }
        }
        // Re-sort after priority adjustment
        std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
            return local_priority.at(a) > local_priority.at(b);
        });
    }

    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == DONE::NOT_DONE) continue;
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env->map.size());
            // ★ 使用 GameOpt 专属 PIBT（内置 agent_blocked）
            gameOptPIBT(i, -1, prev_states_, next_states_,
                        prev_decision_, decision_, occupied_, trajLNS_);
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
void GameOptPlanner::append_actions_and_rollout_states(
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
void GameOptPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                          SharedEnvironment* env, int num_steps)
{
    actions.clear();
    if (env == nullptr || env->num_of_agents <= 0) return;

    actions.resize(env->num_of_agents);
    if (num_steps <= 0) return;

    const auto episode_start = std::chrono::steady_clock::now();

    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    const int flow_budget_ms = std::max(0, time_limit - pibt_time * num_steps - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);
    TimePoint hard_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit - 5));

    std::vector<double> local_priority = p_;

    const std::vector<State> original_states = env->curr_states;
    const int original_timestep = env->curr_timestep;
    const std::vector<DCR> original_decided = decided_;

    decided_.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

    initialize_dummy_goals_if_needed(env);
    setup_multistep_episode_state(env, flow_end_time, local_priority);
    update_guide_paths_once_for_multistep(env, flow_end_time);

    p_ = local_priority;

    for (int step = 0; step < num_steps; step++) {
        if (step > 0 && std::chrono::steady_clock::now() >= hard_deadline) break;

        std::vector<Action> one_step_actions;

        if (step > 0) {
            refresh_multistep_step_state(env, local_priority);
        }

        run_multistep_pibt_once(env, local_priority, one_step_actions);
        append_actions_and_rollout_states(env, actions, one_step_actions);

        last_step_actions_ = one_step_actions;
        last_step_states_ = env->curr_states;
    }

    // 恢复 env state
    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;
}

} // namespace optsvr
