// optsvr/planner/IdleYield/IdleYieldPlanner.h
// 来源: 基于 FlowDecay/OptPlanner 修改
// 核心改动: 空闲 agent (无任务) 的 dummy goal = 当前位置（而非出生位置）
//   → PIBT 自然选择 Wait，但仍参与 push-chain 可被推开让路
//   → 消除了空闲 agent 回出生点造成的路径冲突和拥堵
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "../FlowDecay/TrajLNS.h"
#include "../FlowDecay/Types.h"
#include "../FlowDecay/PibtDensity.h"
#include <vector>
#include <random>

namespace optsvr {

class IdleYieldPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // ============ 持久状态（跨 plan 调用保持） ============
    std::vector<int> decision_;
    std::vector<int> prev_decision_;
    std::vector<double> p_;
    std::vector<double> p_copy_;
    std::vector<State> prev_states_;
    std::vector<State> next_states_;
    std::vector<int> ids_;
    std::vector<bool> occupied_;
    std::vector<OptPlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    OptPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    bool initialized_ = false;

    // PIBT 密度惩罚
    PibtDensity pibt_density_;
    std::vector<Action> last_step_actions_;
    std::vector<State> last_step_states_;

    // Warehouse 通道定向
    std::vector<std::vector<int>> extra_weights_;
    void build_row_column_highway(SharedEnvironment* env);

    // ============ 内部子步骤 ============
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
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
