// optsvr/planner/EPIBT/EPIBTPlanner.h
// 来源: IDEA/GameOpt/MAPFPlanner.cpp
// EPIBT: Enhanced PIBT with turn-cost aware successor sorting
// 对比默认 PIBT：排序时 h_eff = h + turn_cost，避免选择需要多步转弯的方向
// 比 TurnAwarePlanner 更轻量（无需预计算大矩阵），适合大地图
// PLANNER_MODE=4 时使用
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "../FlowDecay/FlowVertexCost.h"
#include "../FlowDecay/Types.h"
#include "../FlowDecay/TrajLNS.h"
#include <vector>
#include <random>
#include <unordered_set>

namespace orz_engine {

class EPIBTPlanner {
public:
    explicit EPIBTPlanner(bool use_lacam_pibt = false);

    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // ═══════════════════════════════════════════════════════════════════
    // EPIBT Core: causalPIBT with turn-cost penalty
    // ═══════════════════════════════════════════════════════════════════
    bool epibt_causal(int curr_id, int higher_id);
    void lacam_pibt_once(std::vector<double>& local_priority);
    bool lacam_func_pibt(int curr_id, int higher_id);
    int lacam_is_swap_required_and_possible(int agent, const std::vector<int>& candidates);
    bool lacam_is_swap_required(int pusher, int puller,
                                int v_pusher_origin, int v_puller_origin);
    bool lacam_is_swap_possible(int v_pusher_origin, int v_puller_origin);
    int lacam_heuristic(int agent, int loc);

    // Turn cost: 0=facing, 1=90°, 2=180°
    static int turn_cost(int curr_ori, int target_ori) {
        if (curr_ori == target_ori) return 0;
        int diff = (target_ori - curr_ori + 4) % 4;
        return (diff <= 2) ? diff : 1;
    }

    static int get_move_orientation(int loc, int next_loc, int cols) {
        int diff = next_loc - loc;
        if (diff == 1) return 0;
        if (diff == cols) return 1;
        if (diff == -1) return 2;
        if (diff == -cols) return 3;
        return -1;
    }

    // 可调参数（环境变量覆盖）
    double turn_cost_mult_ = 1.0;   // OPT_EPIBT_TURN_MULT: turn_cost 乘数
    int    wait_penalty_   = 1;     // OPT_EPIBT_WAIT_PENALTY: 等待额外惩罚
    double hot_priority_boost_ = 0.0;    // OPT_EPIBT_HOT_PRIORITY_BOOST
    double drain_priority_boost_ = 0.0;  // OPT_EPIBT_DRAIN_PRIORITY_BOOST
    double stuck_priority_boost_ = 0.0;  // OPT_EPIBT_STUCK_PRIORITY_BOOST
    int    stuck_cap_ = 0;               // OPT_EPIBT_STUCK_CAP
    double density_penalty_ = 0.0;       // OPT_EPIBT_DENSITY_PENALTY
    int    lacam_mccg_num_ = 1;          // OPT_LACAM_MCCG_NUM
    double lacam_mccg_wait_score_ = 1.0; // OPT_LACAM_MCCG_WAIT_SCORE
    double lacam_mccg_move_bonus_ = 0.0; // OPT_LACAM_MCCG_MOVE_BONUS
    double lacam_mccg_flow_score_ = 0.0; // OPT_LACAM_MCCG_FLOW_SCORE
    double lacam_mccg_priority_noise_ = 0.0; // OPT_LACAM_MCCG_PRIORITY_NOISE
    double lacam_guide_next_bonus_ = 0.0;    // OPT_LACAM_GUIDE_NEXT_BONUS
    bool   lacam_guide_next_first_ = false;  // OPT_LACAM_GUIDE_NEXT_FIRST
    bool   lacam_mccg_active_ = false;
    OptPlan::VertexCostParams flow_vertex_params_;
    bool use_lacam_pibt_ = false;
    bool lacam_swap_enabled_ = true;

    // ═══════════════════════════════════════════════════════════════════
    // 持久状态
    // ═══════════════════════════════════════════════════════════════════
    std::vector<int> decision_;
    std::vector<int> prev_decision_;
    std::vector<double> p_;
    std::vector<double> p_copy_;
    std::vector<State> prev_states_;
    std::vector<State> next_states_;
    std::vector<int> ids_;
    std::vector<bool> occupied_;
    std::vector<int> lacam_occupied_now_;
    std::vector<int> lacam_occupied_next_;
    std::vector<OptPlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    std::vector<int> dummy_goals_;
    std::vector<int> last_locations_;
    std::vector<int> stuck_ticks_;
    OptPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    bool initialized_ = false;
    SharedEnvironment* env_ = nullptr;
    OptPlan::Neighbors neighbors_;
    std::vector<double> flow_vertex_decay_table_{1.0};
    std::vector<std::vector<int>> extra_weights_;  // [loc][direction] guide-path penalty

    // 内部函数
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    double flow_vertex_adjustment(int loc);
    int local_density_count(int loc) const;
    double local_density_adjustment(int loc, bool is_wait) const;
    void refresh_stuck_tracking(const std::vector<State>& states);
    void apply_pibt_priority_adjustments(std::vector<double>& pibt_priority) const;
    int lacam_guide_next_loc(int agent, int loc) const;
    double lacam_candidate_set_score();
    void lacam_pibt_mccg(std::vector<double>& local_priority);
    void build_row_column_highway(SharedEnvironment* env);
    const std::vector<std::vector<int>>* guide_extra_weights() const {
        return extra_weights_.empty() ? nullptr : &extra_weights_;
    }
    void update_guide_paths_for_plan(SharedEnvironment* env, TimePoint flow_end,
                                     int& guide_required, int& guide_completed,
                                     int& guide_failed, bool& guide_stopped_by_time);
    void update_guide_paths_serial(SharedEnvironment* env, TimePoint flow_end,
                                   int& guide_completed, int& guide_failed,
                                   bool& guide_stopped_by_time);
    void update_guide_paths_parallel_rounds(SharedEnvironment* env, TimePoint flow_end,
                                            int& guide_completed, int& guide_failed,
                                            bool& guide_stopped_by_time);
    int run_parallel_guide_round(SharedEnvironment* env, TimePoint flow_end,
                                 const std::vector<int>& round_agents,
                                 int max_threads, int& guide_failed,
                                 bool& guide_stopped_by_time,
                                 std::unordered_set<int>* updated_agents = nullptr);
    void improve_guide_paths_parallel_rounds(SharedEnvironment* env, TimePoint flow_end,
                                             int& improve_completed, int& improve_failed,
                                             bool& improve_stopped_by_time,
                                             int* unique_improved_agents = nullptr);
};

} // namespace orz_engine
