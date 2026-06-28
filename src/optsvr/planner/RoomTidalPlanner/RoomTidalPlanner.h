// optsvr/planner/RoomTidalPlanner/RoomTidalPlanner.h
// Room Tidal Gate-Control Planner — periodic vote tidal priority with refuge layout
// Ported from LORR26_ssfc: room_c_5_6_tfo_periodic_vote_refuge_tidal_priority
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include "../RoomPlanner/TrajLNS.h"
#include "../RoomPlanner/Types.h"
#include "../RoomPlanner/flow.h"
#include "../RoomPlanner/pibt.h"
#include "../RoomPlanner/const.h"
#include "../RoomPlanner/heuristics.h"
#include "../RoomPlanner/utils.h"
#include <vector>
#include <random>

namespace optsvr {

class RoomTidalPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // ═══════════════════════════════════════════════════════════════
    // Strategy constants
    // ═══════════════════════════════════════════════════════════════
    static constexpr int ROOM_C_DOOR_ROW = 16;
    static constexpr int ROOM_C_DOOR_COL = 1;
    static constexpr int ROOM_C_VOTE_BUFFER_ROW_RADIUS = 8;
    static constexpr int ROOM_C_VOTE_BUFFER_OUTWARD_EXTRA = 2;
    static constexpr int ROOM_C_VOTE_BUFFER_COL_MAX = 9;
    static constexpr int ROOM_C_CONTROL_BUFFER_ROW_RADIUS = 4;
    static constexpr int ROOM_C_CONTROL_BUFFER_COL_MAX = 5;
    static constexpr double ROOM_C_DOWN_PRIORITY_DELTA = 1000.0;
    static constexpr int ROOM_A_DOOR_ROW = 20;
    static constexpr int ROOM_A_DOOR_COL = 32;
    static constexpr int ROOM_A_BUFFER_MIN_ROW = 17;
    static constexpr double ROOM_A_RIGHT_PRIORITY_DELTA = 1000.0;
    static constexpr int ROOM_6_10_DUAL_LANE_TOP_ROW = 31;
    static constexpr int ROOM_6_10_DUAL_LANE_WALL_ROW = 32;
    static constexpr int ROOM_6_10_DUAL_LANE_BOTTOM_ROW = 33;
    static constexpr int ROOM_6_10_DOWN_LANE_COL = 44;
    static constexpr int ROOM_6_10_UP_LANE_COL = 47;
    static constexpr int ROOM_10_11_DUAL_LANE_TOP_ROW = 33;
    static constexpr int ROOM_10_11_DUAL_LANE_LEFT_COL = 47;
    static constexpr int ROOM_10_11_DUAL_LANE_WALL_COL = 48;
    static constexpr int ROOM_10_11_DUAL_LANE_RIGHT_COL = 49;
    static constexpr int ROOM_6_7_DUAL_LANE_LOWER_ROW = 29;
    static constexpr int ROOM_6_7_DUAL_LANE_WALL_COL = 48;
    static constexpr int ROOM_6_7_DUAL_LANE_RIGHT_COL = 49;
    static constexpr int ROOM_1_2_DUAL_LANE_TOP_ROW = 11;
    static constexpr int ROOM_1_2_DUAL_LANE_BOTTOM_ROW = 14;
    static constexpr int ROOM_1_2_DUAL_LANE_LEFT_COL = 31;
    static constexpr int ROOM_1_2_DUAL_LANE_WALL_COL = 32;
    static constexpr int ROOM_1_2_DUAL_LANE_RIGHT_COL = 33;
    static constexpr int ROOM_1_2_WAITING_MIN_ROW = 7;
    static constexpr int ROOM_1_2_WAITING_TOP_MAX_ROW = 11;
    static constexpr int ROOM_1_2_WAITING_BOTTOM_MIN_ROW = 12;
    static constexpr int ROOM_1_2_WAITING_MAX_ROW = 15;
    static constexpr int ROOM_1_2_WAITING_MIN_COL = 24;
    static constexpr int ROOM_1_2_WAITING_MAX_COL = 35;
    static constexpr int ROOM_6_10_WAITING_TOP_MIN_ROW = 24;
    static constexpr int ROOM_6_10_WAITING_TOP_MAX_ROW = 31;
    static constexpr int ROOM_6_10_WAITING_BOTTOM_MIN_ROW = 33;
    static constexpr int ROOM_6_10_WAITING_BOTTOM_MAX_ROW = 40;
    static constexpr int ROOM_6_10_WAITING_MIN_COL = 42;
    static constexpr int ROOM_6_10_WAITING_LEFT_MAX_COL = 44;
    static constexpr int ROOM_6_10_WAITING_RIGHT_MIN_COL = 45;
    static constexpr int ROOM_6_10_WAITING_MAX_COL = 47;
    static constexpr int ROOM_WAITING_SIDE_BASE_PENALTY = 80;
    static constexpr int ROOM_WAITING_HALF_CROSS_BASE_PENALTY = 40;
    static constexpr int ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MIN_ROW = 25;
    static constexpr int ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MAX_ROW = 31;
    static constexpr int ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MIN_COL = 43;
    static constexpr int ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MAX_COL = 46;
    static constexpr int ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_PENALTY = 40;
    static constexpr int ROOM_C_REFUGE_ENTRY_PENALTY = 120;
    static constexpr int ROOM_A_REFUGE_ENTRY_PENALTY = 120;
    static constexpr int ROOM_GUIDE_NEXT_STEP_BONUS = 30;
    static constexpr int ROOM_GUIDE_OFF_PATH_PENALTY = 8;
    static constexpr int ROOM_GUIDE_BACKTRACK_PENALTY = 45;
    static constexpr int ROOM_GUIDE_BACKTRACK_STEP_PENALTY = 8;
    static constexpr int ROOM_GUIDE_WAIT_PENALTY = 12;
    static constexpr int ROOM_GUIDE_FORWARD_SKIP_PENALTY = 4;
    static constexpr int ROOM_PARTITION_SIZE = 16;
    static constexpr int ROOM_PARTITION_COUNT = 4;
    static constexpr int ROOM_REGION_COUNT = ROOM_PARTITION_COUNT * ROOM_PARTITION_COUNT;
    static constexpr int BOTTLENECK_HIGHWAY_PENALTY = 40;
    static constexpr int TIDAL_PRIORITY_PERIOD = 80;

    // ═══════════════════════════════════════════════════════════════
    // Persistent state (across plan calls)
    // ═══════════════════════════════════════════════════════════════
    SharedEnvironment* env_ = nullptr;
    std::vector<int> decision_;
    std::vector<int> prev_decision_;
    std::vector<double> priorities_;
    std::vector<double> initial_priorities_;
    std::vector<State> prev_states_;
    std::vector<State> next_states_;
    std::vector<int> ids_;
    std::vector<bool> occupied_;
    std::vector<RoomPlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    std::vector<int> dummy_goals_;
    RoomPlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    std::vector<std::vector<int>> extra_weights_;
    std::vector<int> room_6_10_last_direction_;
    std::vector<int> room_1_2_last_direction_;
    bool initialized_ = false;

    // Tidal priority state
    mutable int room_c_priority_phase_start_ = -1;
    mutable int room_c_preferred_direction_ = 0;
    mutable int room_c_locked_down_vote_count_ = 0;
    mutable int room_c_locked_up_vote_count_ = 0;
    mutable std::vector<int> room_c_locked_agent_votes_;
    mutable std::vector<int> room_c_locked_agent_vote_locs_;
    mutable int room_a_priority_phase_start_ = -1;
    mutable int room_a_preferred_direction_ = 0;
    mutable int room_a_locked_right_vote_count_ = 0;
    mutable int room_a_locked_left_vote_count_ = 0;
    mutable std::vector<int> room_a_locked_agent_votes_;
    mutable std::vector<int> room_a_locked_agent_vote_locs_;

    // ═══════════════════════════════════════════════════════════════
    // Internal methods
    // ═══════════════════════════════════════════════════════════════
    bool is_room_map() const;
    void initialize_extra_weights();
    void build_room_fixed_dual_lane_highway();
    bool update_room_guide_traj(int agent_id, const TimePoint* deadline);
    void frank_wolfe_room_guide(TimePoint timelimit);

    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_dummy_goals_if_needed();
    void setup_multistep_episode_state(TimePoint flow_end_time, std::vector<double>& local_priority);
    void update_guide_paths_once(TimePoint flow_end_time);
    void refresh_multistep_step_state(std::vector<double>& local_priority);
    void run_tidal_pibt_once(std::vector<double>& local_priority, std::vector<Action>& one_step_actions);
    void append_actions_and_rollout_states(std::vector<std::vector<Action>>& actions,
                                           const std::vector<Action>& one_step_actions);

    // Room C tidal direction
    int room_c_down_priority_direction(int agent_id) const;
    int room_c_periodic_preferred_direction() const;
    bool room_c_down_flow_active() const;
    bool room_c_down_priority_forbids_move(int agent_id, int from_loc, int to_loc) const;

    // Room A tidal direction
    int room_a_right_priority_direction(int agent_id) const;
    int room_a_periodic_preferred_direction() const;
    bool room_a_right_flow_active() const;
    bool room_a_right_priority_forbids_move(int agent_id, int from_loc, int to_loc) const;

    // Causal PIBT with room door control
    bool causal_pibt_with_room_c_down_priority(int curr_id, int higher_id,
        std::vector<State>& prev_states, std::vector<State>& next_states,
        std::vector<int>& prev_decision, std::vector<int>& decision,
        std::vector<bool>& occupied, RoomPlan::TrajLNS& lns);
};

} // namespace optsvr
