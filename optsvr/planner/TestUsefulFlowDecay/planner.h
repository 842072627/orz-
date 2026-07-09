// optsvr/planner/TestUsefulFlowDecay/planner.h
// Source: LORR26_YxuanwKeith@test_useful:optsvr/planner/FlowDecay.
// Isolated copy of the test_useful FlowDecay planner exposed as planner_mode=11.
// optplan/planner.h — 优化版 Planner
//
// 复制 DefaultPlanner::plan 的整体逻辑骨架，
// 维护自己的完整状态（TrajLNS、decision、priority 等）。
// guide path 阶段使用串行逻辑（与 DefaultPlanner 行为一致）。
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "TrajLNS.h"
#include "Types.h"
#include "TestUsefulPibtDensity.h"
#include <deque>
#include <vector>
#include <random>
#include <tuple>
#include <utility>

namespace optsvr {

class TestUsefulFlowDecayPlanner {
public:
    // 初始化（在 MAPFPlanner::initialize 中调用一次）
    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    // 每轮规划（在 MAPFPlanner::plan 中调用）
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // ============ 持久状态（跨 plan 调用保持） ============
    std::vector<int> decision_;
    std::vector<int> prev_decision_;
    std::vector<double> p_;         // 全局优先级
    std::vector<double> p_copy_;    // 初始优先级备份
    std::vector<State> prev_states_;
    std::vector<State> next_states_;
    std::vector<int> ids_;
    std::vector<bool> occupied_;
    std::vector<TestUsefulFlowDecayPlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    std::vector<int> idle_spawn_goals_;
    std::vector<int> idle_park_targets_;
    std::vector<int> idle_park_assigned_ticks_;
    std::vector<int> idle_park_claims_;
    std::vector<int> idle_park_occupancy_;
    std::vector<int> idle_park_candidates_;
    std::vector<unsigned char> idle_park_active_;
    std::deque<std::pair<int, std::vector<int>>> idle_park_window_;
    std::vector<int> pibt_dummy_goals_;
    int idle_park_last_tick_ = -1;
    bool idle_park_initialized_ = false;
    TestUsefulFlowDecayPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    std::mt19937 idle_park_rng_;
    bool initialized_ = false;

    // ---- PIBT 密度惩罚系统 ----
    TestUsefulPibtDensity pibt_density_;
    // 上一轮最后一步的动作（用于统计 FW/W 比率更新密度）
    std::vector<Action> last_step_actions_;
    // 上一轮最后一步的 agent 状态
    std::vector<State> last_step_states_;
    // 当前 plan() 内的临时 errand 进度：只用于 multi-step rollout 目标，不写回真实任务。
    std::vector<int> virtual_errand_indices_;
    std::vector<int> virtual_errand_task_ids_;

    // ---- Warehouse 通道定向 (WARE_DIRECT, DHFULL版: 80%阈值 + 软惩罚) ----
    std::vector<std::vector<int>> extra_weights_;  // [loc][direction] penalty
    void build_row_column_highway(SharedEnvironment* env);

    // ============ 内部子步骤（与 DefaultPlanner::planner.cpp 对应） ============
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_idle_spawn_goals_if_needed(SharedEnvironment* env);
    void reset_virtual_errand_state(SharedEnvironment* env);
    int continued_errand_planning_goal(SharedEnvironment* env, int agent_id, bool* advanced);
    void invalidate_agent_guide_distance(int agent_id);
    bool idle_low_load_parking_enabled(const SharedEnvironment* env) const;
    void initialize_idle_low_load_parking_if_needed(SharedEnvironment* env);
    void update_idle_low_load_occupancy(SharedEnvironment* env);
    int pick_low_load_idle_target(SharedEnvironment* env, int agent);
    void set_idle_park_target(int agent, int target, int tick);
    void clear_idle_park_target(int agent);
    bool idle_agent_is_parking_to_target(const SharedEnvironment* env, int agent) const;
    bool idle_agent_is_parked_at_target(const SharedEnvironment* env, int agent) const;
    int idle_target_for_agent(SharedEnvironment* env, int agent);
    void setup_multistep_episode_state(SharedEnvironment* env, TimePoint flow_end_time,
                                       std::vector<double>& local_priority);
    void update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time);
    void refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority);
    void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                 std::vector<Action>& one_step_actions);
    void append_actions_and_rollout_states(SharedEnvironment* env,
                                           std::vector<std::vector<Action>>& actions,
                                           const std::vector<Action>& one_step_actions);

};

} // namespace optsvr
