#include "IronPlatoonExecutor.h"
#include "Executor.h"
#include "IronOpenGridConstraints.h"
#include "../../planner/GameDirectionalPlanner/orz_highway.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <list>

namespace optsvr {

// Constants
static const int TEMPORARY_PASS_MIN_ACTIONS_AWAY = 3;
static const int TEMPORARY_PASS_NOT_REACHED = 1000000000;

static int lacam_extra_steps()
{
    const char* value = std::getenv("OPT_LACAM_EXTRA_STEPS");
    if (value == nullptr || *value == '\0') return 0;
    return std::clamp(std::atoi(value), 0, 16);
}

static bool map_location_is_free(int location, SharedEnvironment* env)
{
    return location >= 0 &&
           location < env->rows * env->cols &&
           location < static_cast<int>(env->map.size()) &&
           env->map[location] == 0;
}

static int free_forward_location(
    int location,
    int orientation,
    SharedEnvironment* env)
{
    if (!map_location_is_free(location, env) ||
        orientation < 0 || orientation >= 4)
    {
        return -1;
    }

    int row = location / env->cols;
    int col = location % env->cols;
    static const int row_delta[4] = {0, 1, 0, -1};
    static const int col_delta[4] = {1, 0, -1, 0};
    int next_row = row + row_delta[orientation];
    int next_col = col + col_delta[orientation];
    if (next_row < 0 || next_row >= env->rows ||
        next_col < 0 || next_col >= env->cols)
    {
        return -1;
    }
    int next_location = next_row * env->cols + next_col;
    return map_location_is_free(next_location, env) ? next_location : -1;
}

static bool platoon_pair_has_safe_alignment(
    int follower,
    int leader,
    const vector<State>& states,
    SharedEnvironment* env)
{
    if (follower < 0 || follower >= static_cast<int>(states.size()) ||
        leader < 0 || leader >= static_cast<int>(states.size()))
    {
        return false;
    }

    const State& follower_state = states[follower];
    const State& leader_state = states[leader];
    if (follower_state.orientation != leader_state.orientation ||
        follower_state.orientation < 0 ||
        follower_state.orientation >= 4 ||
        leader_state.counter.count <= 0 ||
        leader_state.current_action != Action::FW)
    {
        return false;
    }

    int direction = follower_state.orientation;
    int leader_location = free_forward_location(
        follower_state.location, direction, env);
    if (leader_state.location != leader_location)
    {
        return false;
    }

    const int leader_next = free_forward_location(leader_location, direction, env);
    if (leader_next < 0) return false;

    if (cfg().executor_mode == 11) {
        if (!cfg().game_highway_enabled) return false;
        const int scope = std::clamp(cfg().game_platoon_scope, 0, 3);
        if (scope == 0) return false;
        if (scope == 3) return true;

        auto orz_cell_allows = [&](int loc) {
            const uint8_t mask = ORZ_Highway::get_forbid(loc);
            if (mask == 0 || iron_forbid_mask_blocks_direction(mask, direction))
                return false;
            if (scope == 1) {
                // These masks leave exactly one legal cardinal direction.
                return mask == 7 || mask == 11 || mask == 13 || mask == 14;
            }
            return true;
        };
        return orz_cell_allows(follower_state.location) &&
               orz_cell_allows(leader_location) &&
               orz_cell_allows(leader_next);
    }

    // Mode 10 keeps its original Iron alternating-open-grid admission rule.
    const uint8_t follower_mask =
        iron_open_grid_forbid_mask(env, follower_state.location);
    const uint8_t leader_mask =
        iron_open_grid_forbid_mask(env, leader_state.location);
    return follower_mask != 0 && leader_mask != 0 &&
           !iron_forbid_mask_blocks_direction(follower_mask, direction) &&
           !iron_forbid_mask_blocks_direction(leader_mask, direction);
}

static bool platoon_headway_allows_tick(const State& follower, const State& leader)
{
    int follower_max = follower.counter.maxCount;
    int leader_max = leader.counter.maxCount;
    if (follower_max <= 0 || leader_max <= 0)
    {
        return false;
    }

    // Use contact-level headway; ownership and ActionModel remain the final guards.
    return static_cast<long long>(leader.counter.count) * follower_max >=
           static_cast<long long>(follower.counter.count + 1) * leader_max;
}

// ==================== Static helper functions ====================

static int get_next_location(const State& state, SharedEnvironment* env)
{
    int moves[4] = {1, env->cols, -1, -env->cols};
    return state.location + moves[state.orientation];
}

static int planned_actions_to_location(
    int agent_id,
    int target_location,
    const vector<State>& states,
    const vector<vector<Action>>& staged_actions,
    const vector<int>& action_index,
    SharedEnvironment* env)
{
    if (states[agent_id].location == target_location)
    {
        return 0;
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    int location = states[agent_id].location;
    int orientation = states[agent_id].orientation;
    int actions_to_target = 0;

    for (int k = action_index[agent_id]; k < staged_actions[agent_id].size(); k++)
    {
        Action action = staged_actions[agent_id][k];
        actions_to_target++;

        if (action == Action::FW)
        {
            location += moves[orientation];
            if (location == target_location)
            {
                return actions_to_target;
            }
        }
        else if (action == Action::CR)
        {
            orientation = (orientation + 1) % 4;
        }
        else if (action == Action::CCR)
        {
            orientation = (orientation - 1) % 4;
            if (orientation == -1)
                orientation = 3;
        }
    }
    return TEMPORARY_PASS_NOT_REACHED;
}

static int planned_actions_to_leave_location_after_entry(
    int agent_id,
    int entered_location,
    const vector<State>& states,
    const vector<vector<Action>>& staged_actions,
    const vector<int>& action_index,
    SharedEnvironment* env)
{
    int curr_action_index = action_index[agent_id];
    if (curr_action_index >= staged_actions[agent_id].size() ||
        staged_actions[agent_id][curr_action_index] != Action::FW ||
        get_next_location(states[agent_id], env) != entered_location)
    {
        return TEMPORARY_PASS_NOT_REACHED;
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    int location = entered_location;
    int orientation = states[agent_id].orientation;
    int actions_after_entry = 0;

    for (int k = curr_action_index + 1; k < staged_actions[agent_id].size(); k++)
    {
        Action action = staged_actions[agent_id][k];
        actions_after_entry++;

        if (action == Action::FW)
        {
            location += moves[orientation];
            if (location != entered_location)
            {
                return actions_after_entry;
            }
        }
        else if (action == Action::CR)
        {
            orientation = (orientation + 1) % 4;
        }
        else if (action == Action::CCR)
        {
            orientation = (orientation - 1) % 4;
            if (orientation == -1)
                orientation = 3;
        }
    }
    return TEMPORARY_PASS_NOT_REACHED;
}

static bool location_is_currently_empty(
    int location,
    int agent_id,
    const vector<State>& states,
    const vector<unsigned short>& occupancy_count)
{
    if (location < 0 || location >= static_cast<int>(occupancy_count.size()))
    {
        return false;
    }

    int other_occupants = occupancy_count[location];
    if (agent_id >= 0 && agent_id < static_cast<int>(states.size()) &&
        states[agent_id].location == location)
    {
        other_occupants--;
    }
    return other_occupants == 0;
}

static bool promote_temporary_pass_if_safe(
    int location,
    int agent_id,
    const vector<State>& states,
    const vector<vector<Action>>& staged_actions,
    const vector<int>& action_index,
    std::list<int>& visit_order,
    SharedEnvironment* env,
    const vector<unsigned short>& occupancy_count)
{
    if (visit_order.empty())
    {
        return false;
    }

    if (!location_is_currently_empty(
            location, agent_id, states, occupancy_count))
    {
        return false;
    }

    auto agent_visit = visit_order.end();
    for (auto it = visit_order.begin(); it != visit_order.end(); ++it)
    {
        if (*it == agent_id)
        {
            agent_visit = it;
            break;
        }
    }
    if (agent_visit == visit_order.end() || agent_visit == visit_order.begin())
    {
        return false;
    }

    int actions_to_leave = planned_actions_to_leave_location_after_entry(
        agent_id, location, states, staged_actions, action_index, env);
    if (actions_to_leave == TEMPORARY_PASS_NOT_REACHED)
    {
        return false;
    }

    for (auto it = visit_order.begin(); it != agent_visit; ++it)
    {
        int blocking_agent = *it;
        int actions_away = planned_actions_to_location(
            blocking_agent, location, states, staged_actions, action_index, env);
        if (actions_away < TEMPORARY_PASS_MIN_ACTIONS_AWAY ||
            actions_to_leave > actions_away)
        {
            return false;
        }
    }

    visit_order.splice(visit_order.begin(), visit_order, agent_visit);
    return true;
}

// ==================== Executor class methods ====================

void IronPlatoonExecutor::initialize(Executor* exec)
{
    exec_ = exec;
    env = exec->env;
    initialize_impl(0);
}

void IronPlatoonExecutor::initialize_impl(int preprocess_time_limit)
{
    window_size = (int)env->min_planner_communication_time / (env->action_time * env->max_counter) + 1;
    if (cfg().planner_mode == 14)
    {
        const int extra_steps = lacam_extra_steps();
        window_size += extra_steps;
        if (extra_steps > 0)
        {
            std::cerr << "[IronPlatoonExecutor] lacam extra_steps=" << extra_steps
                      << " window=" << window_size << "\n";
        }
    }
    previous_locations.resize(env->num_of_agents, -1);
    last_orientations.resize(env->num_of_agents, -1);
    rotation_in_progress.resize(env->num_of_agents, false);
    fw_pre_approved.resize(env->num_of_agents, false);
    platoon_follow_active.assign(env->num_of_agents, false);
    platoon_follow_start_locations.assign(env->num_of_agents, -1);
    platoon_follow_leaders.assign(env->num_of_agents, -1);
    platoon_prediction_blocked.assign(env->num_of_agents, false);
    platoon_discard_next_plan.assign(env->num_of_agents, false);
    simulation_action_index.assign(env->num_of_agents, 0);
    simulation_previous_locations.assign(env->num_of_agents, -1);
    simulation_projected_locations.assign(env->num_of_agents, -1);
    simulation_projected_orientations.assign(env->num_of_agents, -1);
    simulation_states.resize(env->num_of_agents);

    int map_size = env->rows * env->cols;
    simulated_occupancy_count.assign(map_size, 0);
    simulated_occupancy_touched.clear();
    simulated_occupancy_touched.reserve(std::min(
        static_cast<size_t>(map_size),
        static_cast<size_t>(env->num_of_agents) *
            static_cast<size_t>(window_size + 1)));

    int free_cells = 0;
    for (int location = 0; location < map_size; location++)
    {
        if (map_location_is_free(location, env))
        {
            free_cells++;
        }
    }

    size_t expected_tpg_locations = std::min(
        static_cast<size_t>(free_cells),
        static_cast<size_t>(env->num_of_agents) *
            static_cast<size_t>(window_size + 1));
    tpg.reserve(expected_tpg_locations);
    temp_tpg.reserve(expected_tpg_locations);

    enable_platooning = true;

    if (enable_platooning)
    {
        platoon_occupant.assign(map_size, -1);
        platoon_in_progress_owner.assign(map_size, -1);
        platoon_incoming_or_reserved.assign(map_size, -1);
        platoon_first_child.assign(env->num_of_agents, -1);
        platoon_next_child.assign(env->num_of_agents, -1);
        platoon_promote_on_reach.assign(env->num_of_agents, 0);
        platoon_certified.assign(env->num_of_agents, 0);
        platoon_occupant_touched.clear();
        platoon_in_progress_touched.clear();
        platoon_incoming_touched.clear();
        platoon_occupant_touched.reserve(env->num_of_agents);
        platoon_in_progress_touched.reserve(env->num_of_agents);
        platoon_incoming_touched.reserve(env->num_of_agents * 2);
        platoon_queue.reserve(env->num_of_agents);
    }
}

void IronPlatoonExecutor::reconcile_completed_platoon_follow(int agent_id, int current_location)
{
    int start_location = platoon_follow_start_locations[agent_id];
    auto start_it = tpg.find(start_location);
    if (start_it != tpg.end())
    {
        auto start_visit = std::find(start_it->second.begin(), start_it->second.end(), agent_id);
        if (start_visit != start_it->second.end())
        {
            start_it->second.erase(start_visit);
        }
    }

    auto& current_visits = tpg[current_location];
    auto current_visit = std::find(current_visits.begin(), current_visits.end(), agent_id);
    if (current_visit != current_visits.end())
    {
        current_visits.splice(current_visits.begin(), current_visits, current_visit);
    }
    else
    {
        current_visits.push_front(agent_id);
    }

    if (platoon_prediction_blocked[agent_id])
    {
        predicted_states[agent_id] = env->system_states[agent_id];
        platoon_discard_next_plan[agent_id] = true;
    }
    platoon_prediction_blocked[agent_id] = false;
    platoon_follow_active[agent_id] = false;
    platoon_follow_start_locations[agent_id] = -1;
    platoon_follow_leaders[agent_id] = -1;
}

vector<State> IronPlatoonExecutor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>>& staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    if (predicted_states.size() != env->num_of_agents)
    {
        predicted_states = env->system_states;
    }

    if (first_execution)
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (tpg.find(env->curr_states[i].location) == tpg.end())
            {
                tpg[env->curr_states[i].location].push_back(i);
            }
        }
        first_execution = false;
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        if ((platoon_follow_active[i] || platoon_prediction_blocked[i]) &&
            env->system_states[i].location != platoon_follow_start_locations[i])
        {
            reconcile_completed_platoon_follow(i, env->system_states[i].location);
        }
        previous_locations[i] = env->system_states[i].location;
        last_orientations[i] = env->system_states[i].orientation;
        rotation_in_progress[i] = false;
        fw_pre_approved[i] = false;
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        simulation_projected_locations[i] = predicted_states[i].location;
        simulation_projected_orientations[i] = predicted_states[i].orientation;
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    auto forward_target = [&](int location, int orientation)
    {
        if (orientation < 0 || orientation >= 4 || env->cols <= 0)
        {
            return -1;
        }
        int target = location + moves[orientation];
        if ((orientation == 0 || orientation == 2) &&
            (location < 0 || target < 0 ||
             location / env->cols != target / env->cols))
        {
            return -1;
        }
        return map_location_is_free(target, env) ? target : -1;
    };
    const std::vector<std::vector<Action>>& plan = plan_struct.actions;

    // First append all actions and TPG locations to the window size. Treat an
    // empty or ragged planner result as per-agent empty suffixes.
    int plan_agents = std::min(
        env->num_of_agents,
        static_cast<int>(std::min(plan.size(), staged_actions.size())));
    vector<unsigned char> plan_suffix_valid(plan_agents, 1);
    for (int i = 0; i < plan_agents; i++)
    {
        if (platoon_discard_next_plan[i])
        {
            plan_suffix_valid[i] = 0;
        }
    }
    for (int timestep = 0; timestep < window_size; timestep++)
    {
        for (int i = 0; i < plan_agents; i++)
        {
            if (!plan_suffix_valid[i] ||
                timestep >= static_cast<int>(plan[i].size()))
            {
                continue;
            }
            int new_location = simulation_projected_locations[i];
            int new_orientation = simulation_projected_orientations[i];

            if (plan[i][timestep] == Action::FW)
            {
                new_location = forward_target(new_location, new_orientation);
                if (new_location < 0)
                {
                    plan_suffix_valid[i] = 0;
                    continue;
                }
                tpg[new_location].push_back(i);
            }
            else if (plan[i][timestep] == Action::CR)
            {
                if (new_orientation < 0 || new_orientation >= 4)
                {
                    plan_suffix_valid[i] = 0;
                    continue;
                }
                new_orientation = (new_orientation + 1) % 4;
            }
            else if (plan[i][timestep] == Action::CCR)
            {
                if (new_orientation < 0 || new_orientation >= 4)
                {
                    plan_suffix_valid[i] = 0;
                    continue;
                }
                new_orientation = (new_orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
            }

            simulation_projected_locations[i] = new_location;
            simulation_projected_orientations[i] = new_orientation;

            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
            {
                staged_actions[i].push_back(plan[i][timestep]);
            }
        }
    }
    std::fill(
        platoon_discard_next_plan.begin(),
        platoon_discard_next_plan.end(),
        false);

    // Now run a mcp simulation assuming no delays for a windowed size, and obtain the actions
    std::fill(
        simulation_action_index.begin(),
        simulation_action_index.end(),
        0);
    simulation_states = env->system_states;
    vector<State>& curr_states = simulation_states;
    temp_tpg.clear();
    for (int location : simulated_occupancy_touched)
    {
        simulated_occupancy_count[location] = 0;
    }
    simulated_occupancy_touched.clear();
    for (const State& state : curr_states)
    {
        if (state.location >= 0 &&
            state.location < static_cast<int>(simulated_occupancy_count.size()))
        {
            if (simulated_occupancy_count[state.location] == 0)
            {
                simulated_occupancy_touched.push_back(state.location);
            }
            simulated_occupancy_count[state.location]++;
        }
    }

    for (int t = 0; t < window_size; t++)
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            simulation_previous_locations[i] = curr_states[i].location;
            if (staged_actions[i].size() <= simulation_action_index[i])
            {
                curr_states[i].timestep++;
                continue;
            }

            // Simulate with mcp (no delay)
            int action_index_before = simulation_action_index[i];
            Action curr_action = staged_actions[i][simulation_action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location;
            int new_orientation = curr_orientation;

            // The physical/simulated occupant is the current owner of its cell.
            // Reconcile explicitly: Release builds remove assert expressions,
            // so front() must never be used as the only invariant check.
            auto& current_visits = tpg[curr_location];
            auto current_visit =
                std::find(current_visits.begin(), current_visits.end(), i);
            if (current_visit == current_visits.end())
            {
                current_visits.push_front(i);
            }
            else if (current_visit != current_visits.begin())
            {
                current_visits.splice(
                    current_visits.begin(), current_visits, current_visit);
            }

            if (curr_action == Action::FW)
            {
                new_location = forward_target(curr_location, curr_orientation);
                auto target_tpg = tpg.find(new_location);
                if (new_location >= 0 &&
                    target_tpg != tpg.end() &&
                    !target_tpg->second.empty() &&
                    (target_tpg->second.front() == i ||
                    promote_temporary_pass_if_safe(
                        new_location,
                        i,
                        curr_states,
                        staged_actions,
                        simulation_action_index,
                        target_tpg->second,
                        env,
                        simulated_occupancy_count)))
                {
                    simulation_action_index[i]++;
                }
                else
                {
                    new_location = curr_location;
                }
            }
            else if (curr_action == Action::CR)
            {
                if (curr_orientation >= 0 && curr_orientation < 4)
                {
                    new_orientation = (curr_orientation + 1) % 4;
                    simulation_action_index[i]++;
                }
            }
            else if (curr_action == Action::CCR)
            {
                if (curr_orientation >= 0 && curr_orientation < 4)
                {
                    new_orientation = (curr_orientation - 1) % 4;
                    if (new_orientation == -1)
                        new_orientation = 3;
                    simulation_action_index[i]++;
                }
            }
            if (curr_location != new_location)
            {
                if (curr_location >= 0 &&
                    curr_location < static_cast<int>(simulated_occupancy_count.size()))
                {
                    if (simulated_occupancy_count[curr_location] > 0)
                    {
                        simulated_occupancy_count[curr_location]--;
                    }
                }
                if (new_location >= 0 &&
                    new_location < static_cast<int>(simulated_occupancy_count.size()))
                {
                    if (simulated_occupancy_count[new_location] == 0)
                    {
                        simulated_occupancy_touched.push_back(new_location);
                    }
                    simulated_occupancy_count[new_location]++;
                }
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            if (simulation_action_index[i] > action_index_before)
            {
                curr_states[i].counter.count = 0;
                curr_states[i].current_action = Action::NA;
            }
        }
        // Clear dependency
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (t == 0)
            {
                temp_tpg[simulation_previous_locations[i]].push_back(i);
            }

            int previous_location = simulation_previous_locations[i];
            if (previous_location != curr_states[i].location)
            {
                // A temporary pass may splice another visitor to the front in
                // this same simulated tick. Erase this exact visit; pop_front()
                // can delete the wrong agent and eventually abort on an empty
                // std::list in Release builds.
                auto previous_tpg = tpg.find(previous_location);
                if (previous_tpg != tpg.end())
                {
                    auto visit = std::find(
                        previous_tpg->second.begin(),
                        previous_tpg->second.end(),
                        i);
                    if (visit != previous_tpg->second.end())
                    {
                        previous_tpg->second.erase(visit);
                    }
                }
                temp_tpg[curr_states[i].location].push_back(i);
            }
        }
    }
    auto preserves_active_forward = [&](int agent)
    {
        return platoon_follow_active[agent] &&
               env->system_states[agent].location ==
                   platoon_follow_start_locations[agent] &&
               !staged_actions[agent].empty() &&
               staged_actions[agent].front() == Action::FW;
    };
    auto physically_started_forward = [&](int agent)
    {
        return env->system_states[agent].counter.count > 0 &&
               env->system_states[agent].current_action == Action::FW;
    };

    bool needs_active_recovery = false;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (preserves_active_forward(i))
        {
            platoon_prediction_blocked[i] = false;
            needs_active_recovery =
                needs_active_recovery || simulation_action_index[i] == 0;
        }
    }

    if (needs_active_recovery)
    {
        const int map_size = static_cast<int>(simulated_occupancy_count.size());
        if (static_cast<int>(platoon_occupant.size()) != map_size)
        {
            platoon_occupant.assign(map_size, -1);
            platoon_occupant_touched.clear();
        }
        else
        {
            for (int location : platoon_occupant_touched)
            {
                platoon_occupant[location] = -1;
            }
            platoon_occupant_touched.clear();
        }
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int location = curr_states[i].location;
            if (location < 0 || location >= map_size)
            {
                continue;
            }
            if (platoon_occupant[location] == -1)
            {
                platoon_occupant_touched.push_back(location);
                platoon_occupant[location] = i;
            }
            else if (platoon_occupant[location] != i)
            {
                platoon_occupant[location] = -2;
            }
        }

        if (static_cast<int>(platoon_in_progress_owner.size()) != map_size)
        {
            platoon_in_progress_owner.assign(map_size, -1);
            platoon_in_progress_touched.clear();
        }
        else
        {
            for (int location : platoon_in_progress_touched)
            {
                platoon_in_progress_owner[location] = -1;
            }
            platoon_in_progress_touched.clear();
        }
        auto record_physical_owner = [&](int target, int agent)
        {
            if (target < 0 || target >= map_size)
            {
                return;
            }
            if (platoon_in_progress_owner[target] == -1)
            {
                platoon_in_progress_touched.push_back(target);
                platoon_in_progress_owner[target] = agent;
            }
            else if (platoon_in_progress_owner[target] != agent)
            {
                platoon_in_progress_owner[target] = -2;
            }
        };
        for (int i = 0; i < env->num_of_agents; i++)
        {
            const State& state = env->system_states[i];
            if (state.counter.count > 0 && state.current_action == Action::FW)
            {
                record_physical_owner(
                    free_forward_location(
                        state.location, state.orientation, env),
                    i);
            }
        }
        auto physical_source_entry_is_safe = [&](int agent)
        {
            int source = env->system_states[agent].location;
            if (source < 0 || source >= map_size)
            {
                return false;
            }
            int incoming = platoon_in_progress_owner[source];
            if (incoming == -1)
            {
                return true;
            }
            return incoming >= 0 && incoming < env->num_of_agents &&
                   platoon_follow_active[incoming] &&
                   platoon_follow_leaders[incoming] == agent &&
                   platoon_pair_has_safe_alignment(
                       incoming, agent, env->system_states, env);
        };

        if (static_cast<int>(platoon_certified.size()) != env->num_of_agents)
        {
            platoon_certified.assign(env->num_of_agents, 0);
        }
        else
        {
            std::fill(platoon_certified.begin(), platoon_certified.end(), 0);
        }
        platoon_queue.clear();
        platoon_queue.reserve(env->num_of_agents);

        auto can_project_forward = [&](int agent)
        {
            if (agent < 0 || agent >= env->num_of_agents ||
                simulation_action_index[agent] != 0 ||
                staged_actions[agent].empty() ||
                staged_actions[agent].front() != Action::FW)
            {
                return false;
            }
            int source = curr_states[agent].location;
            int target = forward_target(
                source, curr_states[agent].orientation);
            return source >= 0 && source < map_size && target >= 0 &&
                   simulated_occupancy_count[source] == 1 &&
                   platoon_occupant[source] == agent &&
                   simulated_occupancy_count[target] == 0;
        };
        auto project_forward = [&](int agent)
        {
            int source = curr_states[agent].location;
            int target = forward_target(
                source, curr_states[agent].orientation);
            simulated_occupancy_count[source]--;
            platoon_occupant[source] = -1;
            if (simulated_occupancy_count[target] == 0)
            {
                simulated_occupancy_touched.push_back(target);
            }
            simulated_occupancy_count[target]++;
            if (platoon_occupant[target] == -1)
            {
                platoon_occupant_touched.push_back(target);
            }
            platoon_occupant[target] = agent;
            curr_states[agent].location = target;
            curr_states[agent].counter.count = 0;
            curr_states[agent].current_action = Action::NA;
            simulation_action_index[agent] = 1;

            auto& target_visits = temp_tpg[target];
            auto visit = std::find(
                target_visits.begin(), target_visits.end(), agent);
            if (visit == target_visits.end())
            {
                target_visits.push_back(agent);
            }
            platoon_prediction_blocked[agent] = false;
        };
        auto preserve_unresolved_chain = [&]()
        {
            for (int agent : platoon_queue)
            {
                simulation_action_index[agent] = 1;
                platoon_follow_start_locations[agent] =
                    curr_states[agent].location;
                platoon_prediction_blocked[agent] = true;
            }
        };
        auto recover_committed_chain = [&](int root)
        {
            platoon_queue.clear();
            int agent = root;
            bool reaches_empty_target = false;

            while (agent >= 0 && agent < env->num_of_agents &&
                   !platoon_certified[agent])
            {
                int source = curr_states[agent].location;
                if (simulation_action_index[agent] != 0 ||
                    staged_actions[agent].empty() ||
                    staged_actions[agent].front() != Action::FW ||
                    !physically_started_forward(agent) ||
                    curr_states[agent].location !=
                        env->system_states[agent].location ||
                    source < 0 || source >= map_size ||
                    simulated_occupancy_count[source] != 1 ||
                    platoon_occupant[source] != agent)
                {
                    break;
                }

                int physical_target = free_forward_location(
                    env->system_states[agent].location,
                    env->system_states[agent].orientation,
                    env);
                if (physical_target < 0 ||
                    platoon_in_progress_owner[physical_target] != agent ||
                    !physical_source_entry_is_safe(agent))
                {
                    break;
                }

                platoon_certified[agent] = 1;
                platoon_queue.push_back(agent);
                int target = forward_target(
                    source, curr_states[agent].orientation);
                if (target < 0)
                {
                    break;
                }
                if (simulated_occupancy_count[target] == 0)
                {
                    reaches_empty_target = true;
                    break;
                }
                if (simulated_occupancy_count[target] != 1)
                {
                    break;
                }

                int leader = platoon_occupant[target];
                if (leader < 0 || leader >= env->num_of_agents ||
                    !platoon_pair_has_safe_alignment(
                        agent,
                        leader,
                        curr_states,
                        env))
                {
                    break;
                }
                agent = leader;
            }

            if (reaches_empty_target)
            {
                for (auto it = platoon_queue.rbegin();
                     it != platoon_queue.rend();
                     ++it)
                {
                    // Every successor has just vacated this target, so the
                    // whole committed chain is projected without overlap.
                    project_forward(*it);
                }
            }
            else
            {
                // The physical FW cannot be discarded. Keep the committed
                // prefix paused at its current discrete state until a later
                // rolling plan can resolve the obstruction safely.
                if (platoon_queue.empty())
                {
                    platoon_queue.push_back(root);
                }
                preserve_unresolved_chain();
            }

            for (int chain_agent : platoon_queue)
            {
                platoon_certified[chain_agent] = 0;
            }
        };

        // Resolve already-started motion first. A reverse-ID platoon can need
        // several simultaneous forward projections even when the normal
        // one-pass simulation advanced only its head.
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (preserves_active_forward(i) &&
                simulation_action_index[i] == 0 &&
                physically_started_forward(i))
            {
                recover_committed_chain(i);
            }
        }

        // An unstarted speculative follower is safe to retain only when its
        // target is now empty. Otherwise revoke it and let MCP re-arbitrate.
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (!preserves_active_forward(i) ||
                simulation_action_index[i] != 0)
            {
                continue;
            }
            if (can_project_forward(i))
            {
                project_forward(i);
            }
            else
            {
                platoon_follow_active[i] = false;
                platoon_follow_start_locations[i] = -1;
                platoon_follow_leaders[i] = -1;
                platoon_prediction_blocked[i] = false;
            }
        }
    }

    for (int i = 0; i < env->num_of_agents; i++)
    {
        int actions_to_keep = simulation_action_index[i];
        if (actions_to_keep < static_cast<int>(staged_actions[i].size()))
        {
            staged_actions[i].resize(actions_to_keep);
        }
    }
    predicted_states.swap(simulation_states);
    tpg.swap(temp_tpg);
    temp_tpg.clear();
    return predicted_states;
}

void IronPlatoonExecutor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    // Phase 1: Reconcile TPG with actual system state and track rotation completion
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        int curr_orientation = env->system_states[i].orientation;

        // Detect rotation completion: orientation changed from last recorded value
        if (rotation_in_progress[i] && curr_orientation != last_orientations[i])
        {
            rotation_in_progress[i] = false;
            last_orientations[i] = curr_orientation;
        }

        // Update TPG: only when location actually changed
        if (prev_location != curr_location)
        {
            if ((platoon_follow_active[i] || platoon_prediction_blocked[i]) &&
                curr_location != platoon_follow_start_locations[i])
            {
                reconcile_completed_platoon_follow(i, curr_location);
            }
            else
            {
                auto prev_tpg = tpg.find(prev_location);
                if (prev_tpg != tpg.end())
                {
                    auto& prev_list = prev_tpg->second;
                    for (auto it = prev_list.begin(); it != prev_list.end(); ++it)
                    {
                        if (*it == i)
                        {
                            prev_list.erase(it);
                            break;
                        }
                    }
                }
            }
            previous_locations[i] = curr_location;
        }
    }
    // Phase 2: Pre-approve FW for agents in rotation
    // If the next action after rotation is FW and the target cell is free in TPG,
    // set fw_pre_approved so the FW goes through without re-checking TPG
    int moves[4] = {1, env->cols, -1, -env->cols};
    for (int i = 0; i < env->system_states.size(); i++)
    {
        if (rotation_in_progress[i] && !env->staged_actions[i].empty())
        {
            Action front_action = env->staged_actions[i].front();
            if (front_action == Action::CR || front_action == Action::CCR)
            {
                // Compute what the new orientation would be after this rotation
                int curr_orientation = env->system_states[i].orientation;
                if (curr_orientation < 0 || curr_orientation >= 4)
                {
                    continue;
                }
                int new_orientation;
                if (front_action == Action::CR)
                    new_orientation = (curr_orientation + 1) % 4;
                else
                {
                    new_orientation = (curr_orientation - 1) % 4;
                    if (new_orientation == -1) new_orientation = 3;
                }

                // Check if the staged actions have a FW right after this rotation
                // (index 1 in staged_actions since index 0 is the rotation itself)
                if (env->staged_actions[i].size() >= 2)
                {
                    auto it = env->staged_actions[i].begin();
                    ++it; // skip the rotation action
                    if (*it == Action::FW)
                    {
                        int curr_location = env->system_states[i].location;
                        int next_location = curr_location + moves[new_orientation];
                        auto next_tpg = tpg.find(next_location);
                        if (next_tpg == tpg.end() ||
                            next_tpg->second.empty() ||
                            next_tpg->second.front() == i)
                        {
                            fw_pre_approved[i] = true;
                        }
                    }
                }
            }
        }
    }
    // Phase 3: Issue commands
    for (int i = 0; i < env->system_states.size(); i++)
    {
        mcp(i, agent_command);
    }

    if (enable_platooning)
    {
        apply_platoon_control(agent_command);
    }
}

void IronPlatoonExecutor::apply_platoon_control(std::vector<ExecutionCommand>& agent_command)
{
    const vector<State>& states = env->system_states;
    const int agent_count = static_cast<int>(states.size());
    const int map_size = env->rows * env->cols;

    if (static_cast<int>(platoon_occupant.size()) != map_size)
    {
        platoon_occupant.assign(map_size, -1);
        platoon_in_progress_owner.assign(map_size, -1);
        platoon_incoming_or_reserved.assign(map_size, -1);
        platoon_occupant_touched.clear();
        platoon_in_progress_touched.clear();
        platoon_incoming_touched.clear();
    }
    if (static_cast<int>(platoon_first_child.size()) != agent_count)
    {
        platoon_first_child.assign(agent_count, -1);
        platoon_next_child.assign(agent_count, -1);
        platoon_promote_on_reach.assign(agent_count, 0);
        platoon_certified.assign(agent_count, 0);
        platoon_queue.reserve(agent_count);
    }

    auto sparse_clear = [](vector<int>& owners, vector<int>& touched)
    {
        for (int location : touched)
        {
            owners[location] = -1;
        }
        touched.clear();
    };
    auto sparse_assign = [](
        vector<int>& owners,
        vector<int>& touched,
        int target,
        int agent)
    {
        if (target < 0 || target >= static_cast<int>(owners.size()))
        {
            return;
        }
        if (owners[target] == -1)
        {
            touched.push_back(target);
        }
        owners[target] = agent;
    };
    auto record_incoming_owner = [](
        vector<int>& owners,
        vector<int>& touched,
        int target,
        int agent)
    {
        if (target < 0 || target >= static_cast<int>(owners.size()))
        {
            return;
        }
        if (owners[target] == -1)
        {
            touched.push_back(target);
            owners[target] = agent;
        }
        else if (owners[target] != agent)
        {
            owners[target] = -2;
        }
    };

    sparse_clear(platoon_occupant, platoon_occupant_touched);
    for (int i = 0; i < agent_count; i++)
    {
        if (states[i].location >= 0 && states[i].location < map_size)
        {
            sparse_assign(
                platoon_occupant,
                platoon_occupant_touched,
                states[i].location,
                i);
        }
    }

    auto commanded_forward_target = [&](int agent)
    {
        if (agent_command[agent] != ExecutionCommand::GO)
        {
            return -1;
        }

        const State& state = states[agent];
        bool advances_forward;
        if (state.counter.count > 0 && state.current_action != Action::NA)
        {
            advances_forward = state.current_action == Action::FW;
        }
        else
        {
            advances_forward =
                !env->staged_actions[agent].empty() &&
                env->staged_actions[agent].front() == Action::FW;
        }
        if (!advances_forward || state.orientation < 0 || state.orientation >= 4)
        {
            return -1;
        }
        return free_forward_location(
            state.location, state.orientation, env);
    };
    sparse_clear(
        platoon_in_progress_owner,
        platoon_in_progress_touched);
    for (int agent = 0; agent < agent_count; agent++)
    {
        const State& state = states[agent];
        if (state.counter.count > 0 && state.current_action == Action::FW &&
            state.orientation >= 0 && state.orientation < 4)
        {
            record_incoming_owner(
                platoon_in_progress_owner,
                platoon_in_progress_touched,
                free_forward_location(
                    state.location, state.orientation, env),
                agent);
        }
    }
    auto deactivate = [&](int follower)
    {
        platoon_follow_active[follower] = false;
        platoon_follow_start_locations[follower] = -1;
        platoon_follow_leaders[follower] = -1;
        platoon_prediction_blocked[follower] = false;
    };
    auto has_unique_forward_commitment = [&](int agent)
    {
        if (agent < 0 || agent >= agent_count)
        {
            return false;
        }
        const State& state = states[agent];
        if (state.counter.count <= 0 || state.current_action != Action::FW ||
            state.orientation < 0 || state.orientation >= 4)
        {
            return false;
        }
        int target = free_forward_location(
            state.location, state.orientation, env);
        return target >= 0 && target < map_size &&
               platoon_in_progress_owner[target] == agent;
    };

    // A follower remains guarded across rolling TPG rebuilds until it completes
    // the forward move, including after its physical leader vacates the target.
    for (int follower = 0; follower < agent_count; follower++)
    {
        if (!platoon_follow_active[follower])
        {
            continue;
        }

        if (states[follower].location != platoon_follow_start_locations[follower] ||
            env->staged_actions[follower].empty() ||
            env->staged_actions[follower].front() != Action::FW)
        {
            deactivate(follower);
            continue;
        }

        int direction = states[follower].orientation;
        if (direction < 0 || direction >= 4)
        {
            if (states[follower].counter.count == 0 ||
                states[follower].current_action != Action::FW)
            {
                deactivate(follower);
            }
            agent_command[follower] = ExecutionCommand::STOP;
            continue;
        }
        int target = free_forward_location(
            states[follower].location, direction, env);
        int incoming = target >= 0 && target < map_size
            ? platoon_in_progress_owner[target]
            : -2;
        if (incoming != -1 && incoming != follower)
        {
            if (states[follower].counter.count == 0 ||
                states[follower].current_action != Action::FW)
            {
                deactivate(follower);
            }
            agent_command[follower] = ExecutionCommand::STOP;
            continue;
        }
        int leader = target >= 0 && target < map_size
            ? platoon_occupant[target]
            : -1;
        if (platoon_prediction_blocked[follower] && leader >= 0)
        {
            agent_command[follower] = ExecutionCommand::STOP;
            continue;
        }
        if (leader < 0)
        {
            agent_command[follower] = ExecutionCommand::GO;
            continue;
        }

        bool leader_has_forward =
            !env->staged_actions[leader].empty() &&
            env->staged_actions[leader].front() == Action::FW;
        if (leader != platoon_follow_leaders[follower] ||
            !leader_has_forward ||
            !has_unique_forward_commitment(leader) ||
            !platoon_pair_has_safe_alignment(
                follower, leader, states, env))
        {
            if (states[follower].counter.count == 0 ||
                states[follower].current_action != Action::FW)
            {
                deactivate(follower);
            }
            agent_command[follower] = ExecutionCommand::STOP;
            continue;
        }

        if (platoon_headway_allows_tick(states[follower], states[leader]))
        {
            agent_command[follower] = ExecutionCommand::GO;
        }
        else
        {
            agent_command[follower] = ExecutionCommand::STOP;
        }
    }

    // Certify chains from a physically clear head. This prevents a follower from
    // amplifying a baseline move whose leader is already approaching congestion.
    sparse_clear(
        platoon_incoming_or_reserved,
        platoon_incoming_touched);
    for (int agent = 0; agent < agent_count; agent++)
    {
        const State& state = states[agent];
        if (state.counter.count > 0 && state.current_action == Action::FW &&
            state.orientation >= 0 && state.orientation < 4)
        {
            record_incoming_owner(
                platoon_incoming_or_reserved,
                platoon_incoming_touched,
                free_forward_location(
                    state.location, state.orientation, env),
                agent);
        }
        int target = commanded_forward_target(agent);
        record_incoming_owner(
            platoon_incoming_or_reserved,
            platoon_incoming_touched,
            target,
            agent);
    }

    auto source_entry_is_platoon_safe = [&](int moving_agent)
    {
        int source = states[moving_agent].location;
        if (source < 0 || source >= map_size)
        {
            return false;
        }
        int incoming = platoon_incoming_or_reserved[source];
        if (incoming == -1 || incoming == moving_agent)
        {
            return true;
        }
        return incoming >= 0 && incoming < agent_count &&
               platoon_follow_active[incoming] &&
               platoon_follow_leaders[incoming] == moving_agent &&
               platoon_pair_has_safe_alignment(
                   incoming, moving_agent, states, env) &&
               platoon_headway_allows_tick(
                   states[incoming], states[moving_agent]);
    };

    std::fill(platoon_first_child.begin(), platoon_first_child.end(), -1);
    std::fill(platoon_next_child.begin(), platoon_next_child.end(), -1);
    std::fill(
        platoon_promote_on_reach.begin(),
        platoon_promote_on_reach.end(),
        0);
    std::fill(platoon_certified.begin(), platoon_certified.end(), 0);
    platoon_queue.clear();

    for (int agent = 0; agent < agent_count; agent++)
    {
        if (agent_command[agent] != ExecutionCommand::GO ||
            env->staged_actions[agent].empty() ||
            env->staged_actions[agent].front() != Action::FW)
        {
            continue;
        }
        int direction = states[agent].orientation;
        if (direction < 0 || direction >= 4)
        {
            continue;
        }
        int target = free_forward_location(
            states[agent].location, direction, env);
        if (target >= 0 && target < map_size &&
            platoon_occupant[target] < 0 &&
            platoon_incoming_or_reserved[target] == agent &&
            source_entry_is_platoon_safe(agent))
        {
            platoon_certified[agent] = 1;
            platoon_queue.push_back(agent);
        }
    }

    // Build the fixed dependency graph once. The old repeated full scan only
    // changed certification, so graph reachability computes the same closure.
    for (int follower = 0; follower < agent_count; follower++)
    {
        int leader = -1;
        bool promote_on_reach = false;

        if (platoon_follow_active[follower])
        {
            leader = platoon_follow_leaders[follower];
            if (agent_command[follower] != ExecutionCommand::GO ||
                leader < 0 || leader >= agent_count)
            {
                continue;
            }
        }
        else
        {
            if (agent_command[follower] != ExecutionCommand::STOP ||
                env->staged_actions[follower].empty() ||
                env->staged_actions[follower].front() != Action::FW)
            {
                continue;
            }

            int direction = states[follower].orientation;
            if (direction < 0 || direction >= 4)
            {
                continue;
            }

            int target = free_forward_location(
                states[follower].location, direction, env);
            leader = target >= 0 && target < map_size
                ? platoon_occupant[target]
                : -1;
            int incoming = target >= 0 && target < map_size
                ? platoon_incoming_or_reserved[target]
                : -2;
            if (leader < 0 ||
                (incoming != -1 && incoming != follower) ||
                !source_entry_is_platoon_safe(follower) ||
                env->staged_actions[leader].empty() ||
                env->staged_actions[leader].front() != Action::FW ||
                !has_unique_forward_commitment(leader) ||
                !platoon_pair_has_safe_alignment(
                    follower,
                    leader,
                    states,
                    env) ||
                !platoon_headway_allows_tick(states[follower], states[leader]))
            {
                continue;
            }

            auto tpg_it = tpg.find(target);
            if (tpg_it == tpg.end() || tpg_it->second.size() < 2)
            {
                continue;
            }
            auto visit = tpg_it->second.begin();
            if (*visit != leader || *++visit != follower)
            {
                continue;
            }

            promote_on_reach = true;
        }

        platoon_next_child[follower] = platoon_first_child[leader];
        platoon_first_child[leader] = follower;
        platoon_promote_on_reach[follower] = promote_on_reach ? 1 : 0;
    }

    for (size_t queue_index = 0;
         queue_index < platoon_queue.size();
         queue_index++)
    {
        int leader = platoon_queue[queue_index];
        for (int follower = platoon_first_child[leader];
             follower >= 0;
             follower = platoon_next_child[follower])
        {
            if (platoon_certified[follower])
            {
                continue;
            }

            if (platoon_promote_on_reach[follower])
            {
                int direction = states[follower].orientation;
                int target = direction >= 0 && direction < 4
                    ? free_forward_location(
                        states[follower].location, direction, env)
                    : -1;
                if (target < 0 || target >= map_size ||
                    (platoon_incoming_or_reserved[target] != -1 &&
                     platoon_incoming_or_reserved[target] != follower))
                {
                    continue;
                }
                sparse_assign(
                    platoon_incoming_or_reserved,
                    platoon_incoming_touched,
                    target,
                    follower);
                platoon_follow_active[follower] = true;
                platoon_follow_start_locations[follower] =
                    states[follower].location;
                platoon_follow_leaders[follower] = leader;
                platoon_prediction_blocked[follower] = false;
                agent_command[follower] = ExecutionCommand::GO;
            }

            platoon_certified[follower] = 1;
            platoon_queue.push_back(follower);
        }
    }

    // An admitted follower owns its target until the discrete move completes.
    // Suppress any later rolling-plan command that tries to enter that cell.
    sparse_clear(
        platoon_incoming_or_reserved,
        platoon_incoming_touched);
    for (int follower = 0; follower < agent_count; follower++)
    {
        if (!platoon_follow_active[follower])
        {
            continue;
        }
        int direction = states[follower].orientation;
        if (direction < 0 || direction >= 4)
        {
            continue;
        }
        int target = free_forward_location(
            states[follower].location, direction, env);
        if (target < 0 || target >= map_size)
        {
            continue;
        }
        if (platoon_incoming_or_reserved[target] == -1)
        {
            sparse_assign(
                platoon_incoming_or_reserved,
                platoon_incoming_touched,
                target,
                follower);
        }
        else if (platoon_incoming_or_reserved[target] != follower)
        {
            platoon_incoming_or_reserved[target] = -2;
        }
    }

    // Physical motion has already started and cannot be revoked safely in
    // favor of a speculative entrant. Reserve every committed destination;
    // converging physical moves remain marked as a conflict (-2).
    for (int agent = 0; agent < agent_count; agent++)
    {
        const State& state = states[agent];
        if (state.counter.count <= 0 || state.current_action != Action::FW ||
            state.orientation < 0 || state.orientation >= 4)
        {
            continue;
        }
        int target = free_forward_location(
            state.location, state.orientation, env);
        if (target < 0 || target >= map_size)
        {
            continue;
        }
        int physical_owner = platoon_in_progress_owner[target];
        sparse_assign(
            platoon_incoming_or_reserved,
            platoon_incoming_touched,
            target,
            physical_owner == agent ? agent : -2);
    }

    for (int agent = 0; agent < agent_count; agent++)
    {
        const State& state = states[agent];
        if (state.counter.count > 0 && state.current_action == Action::FW &&
            state.orientation >= 0 && state.orientation < 4)
        {
            int committed_target =
                free_forward_location(
                    state.location, state.orientation, env);
            if (committed_target >= 0 && committed_target < map_size &&
                platoon_occupant[committed_target] < 0)
            {
                int owner = platoon_incoming_or_reserved[committed_target];
                if (owner == -1 &&
                    platoon_in_progress_owner[committed_target] == -2)
                {
                    // Two physical commitments already converge on this cell.
                    // With no active reservation to arbitrate them, advancing
                    // either one can make the continuous states overlap.
                    agent_command[agent] = ExecutionCommand::STOP;
                    continue;
                }
                if (owner == agent)
                {
                    agent_command[agent] = ExecutionCommand::GO;
                    continue;
                }
                if (platoon_in_progress_owner[committed_target] == agent &&
                    owner == -1)
                {
                    agent_command[agent] = ExecutionCommand::GO;
                    continue;
                }
                if (platoon_in_progress_owner[committed_target] == agent &&
                    owner != agent)
                {
                    bool owner_started = owner >= 0 &&
                        states[owner].counter.count > 0 &&
                        states[owner].current_action == Action::FW;
                    if (owner >= 0 && !owner_started)
                    {
                        deactivate(owner);
                        agent_command[owner] = ExecutionCommand::STOP;
                        platoon_incoming_or_reserved[committed_target] = agent;
                        agent_command[agent] = ExecutionCommand::GO;
                    }
                    else
                    {
                        agent_command[agent] = ExecutionCommand::STOP;
                        if (owner >= 0)
                        {
                            agent_command[owner] = ExecutionCommand::GO;
                        }
                    }
                    continue;
                }
            }
        }

        int target = commanded_forward_target(agent);
        if (target < 0 || target >= map_size)
        {
            continue;
        }

        int vacating_owner = platoon_occupant[target];
        if (vacating_owner >= 0 && vacating_owner != agent)
        {
            bool owner_committed =
                platoon_follow_active[vacating_owner] ||
                (states[vacating_owner].counter.count > 0 &&
                 states[vacating_owner].current_action == Action::FW);
            bool aligned_platoon_child =
                platoon_follow_active[agent] &&
                platoon_follow_leaders[agent] == vacating_owner &&
                platoon_pair_has_safe_alignment(
                    agent, vacating_owner, states, env);
            if (owner_committed && !aligned_platoon_child)
            {
                // A moving box owns both its destination and the cell it is
                // sweeping out of. Only its certified same-axis child may enter.
                agent_command[agent] = ExecutionCommand::STOP;
                continue;
            }
        }

        int owner = platoon_incoming_or_reserved[target];
        if (owner != -1 && owner != agent)
        {
            agent_command[agent] = ExecutionCommand::STOP;
        }
    }
}

bool IronPlatoonExecutor::mcp(int agent_id, std::vector<ExecutionCommand>& agent_command)
{
    if (env->staged_actions[agent_id].empty())
    {
        agent_command[agent_id] = ExecutionCommand::STOP;
        rotation_in_progress[agent_id] = false;
        fw_pre_approved[agent_id] = false;
        return false;
    }

    Action front_action = env->staged_actions[agent_id].front();

    // Rotation actions: keep GO until orientation actually changes
    if (front_action == Action::CR || front_action == Action::CCR)
    {
        rotation_in_progress[agent_id] = true;
        agent_command[agent_id] = ExecutionCommand::GO;
        return false;
    }

    // FW with pre-approval: skip TPG check
    if (front_action == Action::FW && fw_pre_approved[agent_id])
    {
        fw_pre_approved[agent_id] = false;
        agent_command[agent_id] = ExecutionCommand::GO;
        return true;
    }

    if (front_action != Action::FW)
    {
        agent_command[agent_id] = ExecutionCommand::GO;
        return false;
    }

    // FW action: check TPG for next location
    int moves[4] = {1, env->cols, -1, -env->cols};
    int curr_location = env->system_states[agent_id].location;
    int curr_orientation = env->system_states[agent_id].orientation;
    if (curr_orientation < 0 || curr_orientation >= 4)
    {
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }
    int next_location = curr_location + moves[curr_orientation];

    auto next_tpg = tpg.find(next_location);
    if (next_tpg == tpg.end() ||
        next_tpg->second.empty() ||
        next_tpg->second.front() == agent_id)
    {
        agent_command[agent_id] = ExecutionCommand::GO;
        return true;
    }
    else
    {
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }
}


} // namespace optsvr
