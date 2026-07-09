// optsvr/planner/TurnAware/TurnAwarePlanner.h
// 转弯感知版 Planner v2：
//   - A* 和 frank_wolfe 使用精确的 turn-aware 距离
//   - PIBT 保持"位置模型"（决定目标格子），但启发式使用 turn-aware 距离
//   - decided_ 扩展为 (loc, orient) 对，正确处理多步转弯
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "../FlowDecay/Types.h"
#include "../FlowDecay/TrajLNS.h"
#include <vector>
#include <random>
#include <queue>

namespace orz_engine {

class TurnAwarePlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // ═══════════════════════════════════════════════════════════════════
    // Turn-Aware Heuristic Table
    // 状态 = (location, orientation)，距离 = 实际 action 步数
    // 预计算：从每个 target 反向 BFS 到所有 (loc, orient) 状态
    // ═══════════════════════════════════════════════════════════════════
    struct TurnAwareHeuristic {
        int n_free = 0;
        int map_size = 0;
        int cols = 0;
        int rows = 0;
        std::vector<int> cell_to_idx;   // location → compressed idx (-1 if wall)
        std::vector<int> idx_to_cell;   // compressed idx → location
        // dist[target_idx][source_idx * 4 + orient] = action 步数
        std::vector<uint16_t> matrix;
        int n_states = 0;               // 4 * n_free
        bool ready = false;

        void build(SharedEnvironment* env);
        // 精确距离: source 面向 source_orient, 到达 target（任意朝向，取最小）
        int get(int source, int source_orient, int target) const;
        // 取 4 朝向最小值（不关心源朝向时用）
        int get_min(int source, int target) const;
    };

    TurnAwareHeuristic ta_heuristic_;

    // ═══════════════════════════════════════════════════════════════════
    // Turn-Aware A* Guide Path 搜索
    // ═══════════════════════════════════════════════════════════════════
    bool ta_astar(SharedEnvironment* env, int start_loc, int start_orient, int goal_loc,
                  std::vector<OptPlan::Int4>& flow, OptPlan::Traj& traj);

    // ═══════════════════════════════════════════════════════════════════
    // PIBT（位置模型 + turn-aware 启发式）
    // 与 FlowDecay 相同的 causalPIBT 架构，只是启发式换成 ta_heuristic
    // ═══════════════════════════════════════════════════════════════════
    bool ta_causalPIBT(int curr_id, int higher_id,
                       std::vector<State>& prev_states,
                       std::vector<State>& next_states,
                       std::vector<int>& prev_decision,
                       std::vector<int>& decision,
                       std::vector<bool>& occupied,
                       OptPlan::TrajLNS& lns);

    int ta_get_gp_h(OptPlan::TrajLNS& lns, int ai, int loc, int orient);

    // ═══════════════════════════════════════════════════════════════════
    // 持久状态（与 OptPlanner 一致的位置模型状态机）
    // ═══════════════════════════════════════════════════════════════════
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
    SharedEnvironment* env_ = nullptr;

    OptPlan::Neighbors neighbors_;

    // ---- 内部子步骤 ----
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_dummy_goals(SharedEnvironment* env);
    void setup_episode(SharedEnvironment* env, TimePoint flow_end,
                       std::vector<double>& local_priority);
    void update_guide_paths(SharedEnvironment* env, TimePoint flow_end);
    void refresh_step(SharedEnvironment* env, std::vector<double>& local_priority);
    void run_pibt_step(SharedEnvironment* env, std::vector<double>& local_priority,
                       std::vector<Action>& one_step_actions);
    void append_actions(SharedEnvironment* env, std::vector<std::vector<Action>>& actions,
                        const std::vector<Action>& one_step_actions);
    void ta_frank_wolfe(OptPlan::TrajLNS& lns, TimePoint timelimit);
};

} // namespace orz_engine
