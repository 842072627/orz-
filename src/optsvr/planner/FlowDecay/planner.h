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
#include "PibtDensity.h"
#include <vector>
#include <random>
#include <tuple>

namespace optsvr {

class OptPlanner {
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
    std::vector<OptPlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    std::vector<int> dummy_goals_;
    OptPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    bool initialized_ = false;

    // ---- PIBT 密度惩罚系统 ----
    PibtDensity pibt_density_;
    // 上一轮最后一步的动作（用于统计 FW/W 比率更新密度）
    std::vector<Action> last_step_actions_;
    // 上一轮最后一步的 agent 状态
    std::vector<State> last_step_states_;
    // 动态 PIBT 时间上限：上一轮实际耗时与 temp 平滑后用于下一轮 guide/PIBT 分配
    int dynamic_pibt_limit_ms_ = -1;
    int dynamic_pibt_static_limit_ms_ = -1;

    // ---- Warehouse 通道定向 (WARE_DIRECT, DHFULL版: 80%阈值 + 软惩罚) ----
    std::vector<std::vector<int>> extra_weights_;  // [loc][direction] penalty
    void build_row_column_highway(SharedEnvironment* env);

    // ============ 内部子步骤（与 DefaultPlanner::planner.cpp 对应） ============
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_dummy_goals_if_needed(SharedEnvironment* env);
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
