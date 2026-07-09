// optsvr/planner/GameOpt/GameOptPlanner.h — Game 算例定制规划器
// 全量包含 FlowDecay 逻辑，不委托 OptPlanner。
// GameOpt 特有策略（PibtLock agent_blocked）直接内联于 PIBT 中。
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "../FlowDecay/TrajLNS.h"
#include "../FlowDecay/Types.h"
#include "../FlowDecay/PibtDensity.h"
#include "../../schedule/GameBalanceOpt/PibtLock.h"
#include <vector>
#include <random>

namespace orz_engine {

class GameOptPlanner {
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
    std::vector<int> dummy_goals_;
    OptPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    bool initialized_ = false;

    // ---- PIBT 密度惩罚系统 ----
    PibtDensity pibt_density_;
    std::vector<Action> last_step_actions_;
    std::vector<State> last_step_states_;

    // ============ 内部子步骤 ============
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

    // ---- GameOpt 专属 PIBT（内置 agent_blocked 逻辑） ----
    bool gameOptPIBT(int curr_id, int higher_id,
                     std::vector<State>& prev_states,
                     std::vector<State>& next_states,
                     std::vector<int>& prev_decision,
                     std::vector<int>& decision,
                     std::vector<bool>& occupied,
                     OptPlan::TrajLNS& lns);
};

} // namespace orz_engine
