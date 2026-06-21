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
#include <chrono>
#include <array>

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
    std::vector<int> last_guide_tick_;  // 每个agent上次拿到guide path的tick (LRU)
    std::vector<int> dummy_goals_;
    OptPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    bool initialized_ = false;

    struct PlanPressureStats {
        int tick = 0;
        int requested_steps = 0;
        int produced_steps = 0;
        int guide_required = 0;
        int guide_replanned = 0;
        int fw_replanned = 0;
        int flow_budget_ms = 0;
        int pibt_reserved_ms = 0;
        int deadline_margin_ms = 0;
        double recent_pibt_avg_ms = 0.0;
        double recent_pibt_max_ms = 0.0;
        double setup_ms = 0.0;
        double guide_ms = 0.0;
        double fw_ms = 0.0;
        double pibt_ms = 0.0;
        double total_ms = 0.0;
        int action_samples = 0;
        int busy_samples = 0;
        int active_samples = 0;
        int fw_samples = 0;
        int turn_samples = 0;
        int wait_samples = 0;
        int active_wait_samples = 0;
        int progress_samples = 0;
        int regress_samples = 0;
        int no_progress_samples = 0;
        int h_samples = 0;
        long long h_delta_sum = 0;
        int density_steps = 0;
        int density_max_cell_sum = 0;
        int density_max_cell_peak = 0;
        int hot_cell_samples = 0;
        int hot_agent_samples = 0;
        int hot_wait_samples = 0;
        // WAIT 原因分解
        int wait_no_successor = 0;  // causalPIBT 所有候选被占
        int wait_cycle = 0;          // moveCheck 循环检测
        int wait_idle = 0;           // 空闲无任务
        int wait_at_goal = 0;        // 已在目标位置
    };
    PlanPressureStats pressure_stats_;
    double pibt_ms_ema_ = 0.0;
    double pibt_ms_peak_ = 0.0;
    std::array<double, 6> recent_pibt_ms_{};
    int recent_pibt_count_ = 0;
    int recent_pibt_next_ = 0;

    // ---- PIBT 密度惩罚系统 ----
    PibtDensity pibt_density_;
    // 上一轮最后一步的动作（用于统计 FW/W 比率更新密度）
    std::vector<Action> last_step_actions_;
    // 上一轮最后一步的 agent 状态
    std::vector<State> last_step_states_;

    // Lane exit buffer zone: non-lane cells within EXIT_BUFFER_RADIUS of any lane cell
    std::vector<bool> lane_exit_buffer_;

    // Highway stall tracking: how many consecutive ticks an agent is stuck on a highway cell
    std::vector<int> highway_stall_ticks_;
    // Previous locations for stall detection
    std::vector<int> prev_locs_;

    // ---- Warehouse 通道定向 (WARE_DIRECT, DHFULL版: 80%阈值 + 软惩罚) ----
    std::vector<std::vector<int>> extra_weights_;  // [loc][direction] penalty
    void build_row_column_highway(SharedEnvironment* env);
    void build_iron_highway(SharedEnvironment* env);

    // ============ 内部子步骤（与 DefaultPlanner::planner.cpp 对应） ============
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_dummy_goals_if_needed(SharedEnvironment* env);
    void setup_multistep_episode_state(SharedEnvironment* env, TimePoint flow_end_time,
                                       std::vector<double>& local_priority);
    void update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time);
    void log_plan_pressure_if_needed(SharedEnvironment* env) const;
    double recent_pibt_avg_ms() const;
    double recent_pibt_max_ms() const;
    void record_pibt_ms(double pibt_ms);
    void refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority);
    void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                 std::vector<Action>& one_step_actions);
    void accumulate_congestion_stats(SharedEnvironment* env,
                                     const std::vector<Action>& one_step_actions);
    void append_actions_and_rollout_states(SharedEnvironment* env,
                                           std::vector<std::vector<Action>>& actions,
                                           const std::vector<Action>& one_step_actions);

};

} // namespace optsvr
