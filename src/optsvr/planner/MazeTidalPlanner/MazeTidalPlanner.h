// optsvr/planner/MazeTidalPlanner/MazeTidalPlanner.h
// Source: IDEA/MAZEOpt default planner, class-wrapped for MAIN2026.
// Maze wall-follow highway + bottom-door tidal priority planner.
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"

// Local MazePlan engine copied into P8 so MazeTidalPlanner does not call RoomPlan.
#include "TrajLNS.h"
#include "Types.h"
#include "flow.h"
#include "pibt.h"
#include "const.h"
#include "heuristics.h"
#include "utils.h"

#include <random>
#include <vector>

namespace optsvr {

class MazeTidalPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    // Constants copied from IDEA/MAZEOpt's maze_wall_follow_vote_tidal_highway defaults.
    static constexpr int MAZE_BOTTOM_VOTE_MIN_ROW = 25;
    static constexpr int MAZE_BOTTOM_VOTE_MAX_ROW = 31;
    static constexpr int MAZE_BOTTOM_VOTE_MIN_COL = 7;
    static constexpr int MAZE_BOTTOM_VOTE_MAX_COL = 31;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_LEFT_MIN_ROW = 25;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_LEFT_MAX_ROW = 31;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_LEFT_MIN_COL = 7;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_LEFT_MAX_COL = 9;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_RIGHT_MIN_ROW = 25;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_RIGHT_MAX_ROW = 31;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_RIGHT_MIN_COL = 29;
    static constexpr int MAZE_BOTTOM_VOTE_EXCLUDE_RIGHT_MAX_COL = 31;
    static constexpr int MAZE_BOTTOM_LEFT_COL = 16;
    static constexpr int MAZE_BOTTOM_RIGHT_COL = 17;
    static constexpr int MAZE_BOTTOM_DOOR_ROW = 31;
    static constexpr int MAZE_BOTTOM_WEST_DOOR_COL = 15;
    static constexpr int MAZE_BOTTOM_EAST_DOOR_COL = 18;
    static constexpr int MAZE_BOTTOM_THIRD_DOOR_COL = 21;
    static constexpr int MAZE_BOTTOM_EAST_REFUGE_COL = 22;
    static constexpr int MAZE_BOTTOM_EAST_REFUGE_MIN_ROW = 28;
    static constexpr int MAZE_BOTTOM_EAST_REFUGE_MAX_ROW = 31;
    static constexpr int MAZE_RIGHT_HAND_HIGHWAY_PENALTY = 240;
    static constexpr int MAZE_CENTRAL_BOTTOM_SPLIT_PENALTY = 240;
    static constexpr int MAZE_RIGHT_HAND_OFFWALL_PENALTY = 1;
    static constexpr int MAZE_BOTTOM_PRIORITY_PERIOD = 80;
    static constexpr double MAZE_BOTTOM_TIDAL_PRIORITY_DELTA = 60.0;
    static constexpr double MAZE_BOTTOM_COL17_FRONT_PRIORITY_STEP = 20.0;
    static constexpr double MAZE_BOTTOM_EXIT_REVERSE_PRIORITY_BOOST = 200000.0;
    static constexpr int ROOM_GUIDE_NEXT_STEP_BONUS = 30;
    static constexpr int ROOM_GUIDE_OFF_PATH_PENALTY = 45;
    static constexpr int ROOM_GUIDE_BACKTRACK_PENALTY = 45;
    static constexpr int ROOM_GUIDE_BACKTRACK_STEP_PENALTY = 8;
    static constexpr int ROOM_GUIDE_WAIT_PENALTY = 8;
    static constexpr int ROOM_GUIDE_FORWARD_SKIP_PENALTY = 4;

    SharedEnvironment* env_ = nullptr;
    std::vector<int> decision_;
    std::vector<int> prev_decision_;
    std::vector<double> priorities_;
    std::vector<double> initial_priorities_;
    std::vector<State> prev_states_;
    std::vector<State> next_states_;
    std::vector<int> ids_;
    std::vector<bool> occupied_;
    std::vector<MazePlan::DCR> decided_;
    std::vector<bool> checked_;
    std::vector<bool> require_guide_path_;
    std::vector<int> dummy_goals_;
    MazePlan::TrajLNS trajLNS_;
    std::mt19937 mt1_;
    std::vector<std::vector<int>> extra_weights_;
    bool initialized_ = false;

    mutable int maze_bottom_east_priority_phase_start_ = -1;
    mutable int maze_bottom_east_preferred_direction_ = 0;
    mutable int maze_bottom_east_locked_right_vote_count_ = 0;
    mutable int maze_bottom_east_locked_left_vote_count_ = 0;
    mutable std::vector<int> maze_bottom_locked_agent_votes_;
    mutable std::vector<int> maze_bottom_locked_agent_vote_locs_;
    mutable std::vector<int> maze_bottom_locked_agent_vote_groups_;

    bool is_maze_map() const;
    void initialize_extra_weights();
    void build_maze_wall_follow_highway();
    bool update_maze_guide_traj(int agent_id, const TimePoint* deadline);
    void frank_wolfe_maze_guide(TimePoint timelimit);

    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env);
    void initialize_dummy_goals_if_needed();
    void setup_multistep_episode_state(TimePoint flow_end_time, std::vector<double>& local_priority);
    void update_guide_paths_once(TimePoint flow_end_time);
    void refresh_multistep_step_state(std::vector<double>& local_priority);
    void run_tidal_pibt_once(std::vector<double>& local_priority, std::vector<Action>& one_step_actions);
    void append_actions_and_rollout_states(std::vector<std::vector<Action>>& actions,
                                           const std::vector<Action>& one_step_actions);

    int maze_bottom_vote_direction(int agent_id) const;
    int maze_bottom_vote_group(int agent_id) const;
    int maze_bottom_periodic_preferred_direction(int vote_group) const;
    bool maze_bottom_clearance_has_same_direction_agent(int vote_group,
                                                        int direction,
                                                        int except_agent,
                                                        int agent_direction) const;
    void apply_maze_bottom_vote_tidal_priority(std::vector<double>& pibt_priority) const;
    bool maze_bottom_priority_forbids_move(int agent_id, int from_loc, int to_loc) const;
    int maze_bottom_east_refuge_retreat_pibt_adjustment(int agent_id, int higher_id,
                                                        int from_loc, int to_loc) const;

    bool causal_pibt_with_extra_weights(int curr_id, int higher_id,
        std::vector<State>& prev_states, std::vector<State>& next_states,
        std::vector<int>& prev_decision, std::vector<int>& decision,
        std::vector<bool>& occupied, MazePlan::TrajLNS& lns);
};

} // namespace optsvr
