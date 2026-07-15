#pragma once
#include "ActionModel.h"
#include "SharedEnv.h"
#include "Plan.h"
#include <unordered_map>
#include <list>
#include <unordered_set>

class Executor;

namespace optsvr {

class IronPlatoonExecutor
{
public:

    SharedEnvironment* env;
    IronPlatoonExecutor() = default;
    void initialize(Executor* exec);
    bool is_initialized() const { return exec_ != nullptr; }   
    
    void initialize_impl(int preprocess_time_limit);
    std::vector<State> process_new_plan(int sync_time_limit, Plan & plan, vector<vector<Action>> & staged_actions);
    void next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command);

    std::unordered_map<int, std::list<int>> tpg;
    std::unordered_map<int, std::list<int>> temp_tpg;

    vector<int> previous_locations;
    vector<int> last_orientations;
    vector<bool> rotation_in_progress;
    vector<bool> fw_pre_approved;
    vector<bool> platoon_follow_active;
    vector<int> platoon_follow_start_locations;
    vector<int> platoon_follow_leaders;
    vector<bool> platoon_prediction_blocked;
    vector<bool> platoon_discard_next_plan;
    vector<int> platoon_occupant;
    vector<int> platoon_in_progress_owner;
    vector<int> platoon_incoming_or_reserved;
    vector<int> platoon_occupant_touched;
    vector<int> platoon_in_progress_touched;
    vector<int> platoon_incoming_touched;
    vector<int> platoon_first_child;
    vector<int> platoon_next_child;
    vector<unsigned char> platoon_promote_on_reach;
    vector<unsigned char> platoon_certified;
    vector<int> platoon_queue;
    vector<unsigned short> simulated_occupancy_count;
    vector<int> simulated_occupancy_touched;
    vector<int> simulation_action_index;
    vector<int> simulation_previous_locations;
    vector<int> simulation_projected_locations;
    vector<int> simulation_projected_orientations;
    vector<State> simulation_states;
    vector<State> predicted_states;

    int window_size;
    bool enable_platooning = false;
    bool first_execution = true;

private:
    Executor* exec_ = nullptr;
    bool mcp(int agent_id, std::vector<ExecutionCommand> & agent_command);
    void apply_platoon_control(std::vector<ExecutionCommand>& agent_command);
    void reconcile_completed_platoon_follow(int agent_id, int current_location);

    // Lightweight micro-simulation for GO validation
    struct SimAgentState {
        int location;
        int orientation;
    };

    // Deterministic 1-step simulation to validate GO set
    // Returns true if the GO set is conflict-free for this timestep
    bool simulate_go_set(const vector<int>& go_candidates,
                         const vector<vector<Action>>& staged_actions,
                         const vector<int>& action_indices,
                         vector<int>& conflict_agents,
                         std::unordered_map<int, std::list<int>>& local_tpg);

    // Check vertex conflict: two agents want the same location
    bool check_vertex_conflict(int agent_a, int agent_b,
                                int loc_a, int loc_b);

    // Check edge conflict: agents swap locations
    bool check_edge_conflict(int agent_a, int agent_b,
                              int loc_a_prev, int loc_a_next,
                              int loc_b_prev, int loc_b_next);

    // Check occupied-target conflict: agent wants to move to a location
    // where another agent has finished its task (target)
    bool check_occupied_target_conflict(int moving_agent, int target_loc,
                                         int stationary_agent,
                                         const vector<vector<pair<int, int>>>& goal_locations);
};

} // namespace optsvr
