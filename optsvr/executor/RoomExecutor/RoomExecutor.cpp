// optsvr/executor/RoomExecutor/RoomExecutor.cpp
// 来源: IDEA/room-0628 dh_executor_occupancy_only_following_preview
// Room 算例专用执行器：occupancy-only preview + following
#include "RoomExecutor.h"
#include "RuntimeConfig.h"
#include <algorithm>

namespace optsvr {

namespace {

void push_front_unique_agent(std::list<int>& queue, int agent_id)
{
    queue.remove(agent_id);
    queue.push_front(agent_id);
}

void push_back_unique_agent(std::list<int>& queue, int agent_id)
{
    if (std::find(queue.begin(), queue.end(), agent_id) == queue.end())
        queue.push_back(agent_id);
}

bool forward_target_is_walkable(
    int location,
    int orientation,
    const SharedEnvironment* env,
    int* target_location)
{
    if (env == nullptr ||
        orientation < 0 ||
        orientation >= 4 ||
        env->rows <= 0 ||
        env->cols <= 0 ||
        location < 0 ||
        location >= static_cast<int>(env->map.size()))
        return false;

    const int row = location / env->cols;
    const int col = location % env->cols;
    int next_row = row;
    int next_col = col;
    if (orientation == 0) next_col++;
    else if (orientation == 1) next_row++;
    else if (orientation == 2) next_col--;
    else next_row--;

    if (next_row < 0 || next_row >= env->rows || next_col < 0 || next_col >= env->cols)
        return false;

    const int next_location = next_row * env->cols + next_col;
    if (next_location < 0 ||
        next_location >= static_cast<int>(env->map.size()) ||
        env->map[next_location] != 0)
        return false;

    if (target_location != nullptr) *target_location = next_location;
    return true;
}

void add_agent_current_occupancy_to_tpg(
    std::unordered_map<int, std::list<int>>& graph,
    const State& state,
    int agent_id,
    int cols)
{
    push_front_unique_agent(graph[state.location], agent_id);

    if (state.current_action == Action::FW &&
        state.counter.count > 0 &&
        state.orientation >= 0 &&
        state.orientation < 4)
    {
        int moves[4] = {1, cols, -1, -cols};
        push_front_unique_agent(graph[state.location + moves[state.orientation]], agent_id);
    }
}

void prioritize_current_occupancy_in_tpg(
    std::unordered_map<int, std::list<int>>& graph,
    const std::vector<State>& states,
    int cols)
{
    for (int i = 0; i < static_cast<int>(states.size()); i++)
        add_agent_current_occupancy_to_tpg(graph, states[i], i, cols);
}

bool staged_action_front_is_valid_from_state(
    const State& state,
    const std::vector<Action>& actions,
    const SharedEnvironment* env)
{
    if (actions.empty() || env == nullptr) return true;

    const Action action = actions.front();
    if (action == Action::CR || action == Action::CCR ||
        action == Action::W || action == Action::NA)
        return true;

    return action == Action::FW &&
           forward_target_is_walkable(state.location, state.orientation, env, nullptr);
}

std::vector<int> unique_locations(std::vector<int> locations)
{
    std::sort(locations.begin(), locations.end());
    locations.erase(std::unique(locations.begin(), locations.end()), locations.end());
    return locations;
}

std::vector<int> current_footprint(const State& state, const SharedEnvironment* env)
{
    std::vector<int> locations;
    if (env == nullptr ||
        state.location < 0 ||
        state.location >= static_cast<int>(env->map.size()))
        return locations;

    locations.push_back(state.location);
    if (state.current_action == Action::FW &&
        state.counter.count > 0 &&
        state.orientation >= 0 &&
        state.orientation < 4)
    {
        int next_location = state.location;
        if (forward_target_is_walkable(state.location, state.orientation, env, &next_location))
            locations.push_back(next_location);
    }

    return unique_locations(std::move(locations));
}

bool location_is_occupied_by_other_footprint(
    int location,
    int agent_id,
    const std::vector<State>& states,
    const SharedEnvironment* env)
{
    for (int i = 0; i < static_cast<int>(states.size()); i++) {
        if (i == agent_id) continue;
        const std::vector<int> footprint = current_footprint(states[i], env);
        if (std::find(footprint.begin(), footprint.end(), location) != footprint.end())
            return true;
    }
    return false;
}

} // namespace

// ==================== 公共接口 ====================

void RoomExecutor::initialize(Executor* exec)
{
    exec_ = exec;
    env_ = exec->env;
    const int action_duration_ms = std::max(1, env_->action_time * env_->max_counter);
    if (optsvr::cfg().dh_executor_ceil_window) {
        window_size_ = (env_->min_planner_communication_time + action_duration_ms - 1) /
                       action_duration_ms + 1;
    } else {
        window_size_ = (int)env_->min_planner_communication_time / action_duration_ms + 1;
    }
    exec_->window_size = window_size_;
    tpg_.clear();
    temp_tpg_.clear();
    predicted_states_.clear();
}

std::vector<State> RoomExecutor::process_new_plan(
    int sync_time_limit, Plan& plan_struct,
    std::vector<std::vector<Action>>& staged_actions)
{
    (void)sync_time_limit;
    if ((int)predicted_states_.size() != env_->num_of_agents) {
        predicted_states_ = env_->system_states;
    }

    const auto& plan = plan_struct.actions;
    if (plan.empty()) return predicted_states_;
    if ((int)staged_actions.size() < env_->num_of_agents)
        staged_actions.resize(env_->num_of_agents);

    auto curr_states = predicted_states_;

    for (int timestep = 0; timestep < (int)plan[0].size() && timestep < window_size_; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            bool append_action = plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W;

            if (plan[i][timestep] == Action::FW) {
                if (!forward_target_is_walkable(
                        curr_states[i].location, curr_states[i].orientation, env_, &new_location))
                {
                    new_location = curr_states[i].location;
                    append_action = false;
                }
            } else if (plan[i][timestep] == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (plan[i][timestep] == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
            }

            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;

            if (append_action) {
                staged_actions[i].push_back(plan[i][timestep]);
            }
        }
    }

    std::vector<int> action_index(env_->num_of_agents, 0);
    curr_states = env_->system_states;
    temp_tpg_.clear();

    for (int t = 0; t < window_size_; t++) {
        auto pre_states = curr_states;
        std::vector<int> reserved_next(env_->map.size(), -1);
        std::vector<bool> action_consumed(env_->num_of_agents, false);
        std::vector<bool> moved_forward(env_->num_of_agents, false);

        bool made_progress = true;
        while (made_progress) {
            made_progress = false;
            for (int i = 0; i < env_->num_of_agents; i++) {
                if (action_consumed[i] ||
                    (int)staged_actions[i].size() <= action_index[i])
                    continue;

                Action curr_action = staged_actions[i][action_index[i]];
                int curr_location = curr_states[i].location;
                int curr_orientation = curr_states[i].orientation;
                int new_location = curr_location;

                if (curr_action == Action::FW) {
                    if (!forward_target_is_walkable(curr_location, curr_orientation, env_, &new_location) ||
                        (new_location >= 0 &&
                         new_location < (int)reserved_next.size() &&
                         reserved_next[new_location] != -1))
                        continue;

                    int leader = -1;
                    for (int candidate = 0; candidate < env_->num_of_agents; candidate++) {
                        if (candidate != i && pre_states[candidate].location == new_location) {
                            leader = candidate;
                            break;
                        }
                    }

                    const bool occupied =
                        location_is_occupied_by_other_footprint(new_location, i, curr_states, env_);
                    const bool leader_vacated =
                        leader >= 0 && curr_states[leader].location != new_location;
                    const bool can_follow_leader =
                        leader_vacated && moved_forward[leader] &&
                        pre_states[leader].orientation == curr_orientation &&
                        curr_states[leader].location != curr_location;
                    if (leader_vacated && !can_follow_leader) continue;
                    if (occupied) continue;

                    if (new_location >= 0 && new_location < (int)reserved_next.size())
                        reserved_next[new_location] = i;
                    curr_states[i].location = new_location;
                    action_index[i]++;
                    action_consumed[i] = true;
                    moved_forward[i] = true;
                    made_progress = true;
                } else if (curr_action == Action::CR) {
                    curr_states[i].orientation = (curr_orientation + 1) % 4;
                    action_index[i]++;
                    action_consumed[i] = true;
                    made_progress = true;
                } else if (curr_action == Action::CCR) {
                    curr_states[i].orientation = (curr_orientation + 3) % 4;
                    action_index[i]++;
                    action_consumed[i] = true;
                    made_progress = true;
                }
            }
        }

        for (int i = 0; i < env_->num_of_agents; i++) {
            curr_states[i].timestep++;
        }

        for (int i = 0; i < env_->num_of_agents; i++) {
            if (t == 0) {
                add_agent_current_occupancy_to_tpg(temp_tpg_, pre_states[i], i, env_->cols);
            }
            if (pre_states[i].location != curr_states[i].location) {
                push_back_unique_agent(temp_tpg_[curr_states[i].location], i);
            }
        }
    }

    for (int i = 0; i < env_->num_of_agents; i++) {
        if (action_index[i] < (int)staged_actions[i].size()) {
            staged_actions[i].resize(action_index[i]);
        }
    }

    predicted_states_ = curr_states;
    tpg_ = temp_tpg_;
    prioritize_current_occupancy_in_tpg(tpg_, env_->system_states, env_->cols);
    return predicted_states_;
}

void RoomExecutor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    (void)exec_time_limit;
    if ((int)agent_command.size() < env_->num_of_agents)
        agent_command.resize(env_->num_of_agents);

    std::vector<int> reserved_next(env_->map.size(), -1);
    std::vector<int> command_order;
    command_order.reserve(env_->system_states.size());
    for (int i = 0; i < (int)env_->system_states.size(); i++) {
        if (!env_->system_states[i].delay.inDelay) command_order.push_back(i);
    }
    for (int i = 0; i < (int)env_->system_states.size(); i++) {
        if (env_->system_states[i].delay.inDelay) command_order.push_back(i);
    }

    for (int i : command_order) {
        const State& state = env_->system_states[i];
        if (state.current_action != Action::NA && state.counter.count > 0) {
            agent_command[i] = ExecutionCommand::GO;
            continue;
        }

        if (env_->staged_actions[i].empty()) {
            agent_command[i] = ExecutionCommand::STOP;
            continue;
        }

        if (env_->staged_actions[i].front() != Action::FW) {
            agent_command[i] = ExecutionCommand::GO;
            continue;
        }

        if (!staged_action_front_is_valid_from_state(state, env_->staged_actions[i], env_)) {
            agent_command[i] = ExecutionCommand::STOP;
            continue;
        }

        int next_location = state.location;
        if (!forward_target_is_walkable(state.location, state.orientation, env_, &next_location) ||
            location_is_occupied_by_other_footprint(next_location, i, env_->system_states, env_) ||
            (next_location >= 0 &&
             next_location < (int)reserved_next.size() &&
             reserved_next[next_location] != -1))
        {
            agent_command[i] = ExecutionCommand::STOP;
            continue;
        }

        if (next_location >= 0 && next_location < (int)reserved_next.size())
            reserved_next[next_location] = i;
        agent_command[i] = ExecutionCommand::GO;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < (int)env_->system_states.size(); i++) {
            const State& follower = env_->system_states[i];
            if (agent_command[i] == ExecutionCommand::GO ||
                follower.delay.inDelay ||
                env_->staged_actions[i].empty() ||
                env_->staged_actions[i].front() != Action::FW ||
                !staged_action_front_is_valid_from_state(follower, env_->staged_actions[i], env_))
                continue;

            int next_location = follower.location;
            if (!forward_target_is_walkable(follower.location, follower.orientation, env_, &next_location))
                continue;

            int leader = -1;
            for (int candidate = 0; candidate < (int)env_->system_states.size(); candidate++) {
                if (candidate != i && env_->system_states[candidate].location == next_location) {
                    leader = candidate;
                    break;
                }
            }
            if (leader < 0 || leader >= (int)agent_command.size() ||
                agent_command[leader] != ExecutionCommand::GO)
                continue;

            const State& leader_state = env_->system_states[leader];
            const bool leader_moves_forward =
                (leader_state.current_action == Action::FW && leader_state.counter.count > 0) ||
                (!env_->staged_actions[leader].empty() &&
                 env_->staged_actions[leader].front() == Action::FW &&
                 staged_action_front_is_valid_from_state(
                     leader_state, env_->staged_actions[leader], env_));
            int leader_next_location = leader_state.location;
            if (!leader_moves_forward ||
                leader_state.delay.inDelay ||
                leader_state.orientation != follower.orientation ||
                !forward_target_is_walkable(
                    leader_state.location, leader_state.orientation, env_, &leader_next_location) ||
                leader_next_location == follower.location ||
                (leader_next_location >= 0 &&
                 leader_next_location < (int)reserved_next.size() &&
                 reserved_next[leader_next_location] != -1 &&
                 reserved_next[leader_next_location] != leader) ||
                (next_location >= 0 &&
                 next_location < (int)reserved_next.size() &&
                 reserved_next[next_location] != -1))
                continue;

            reserved_next[next_location] = i;
            agent_command[i] = ExecutionCommand::GO;
            changed = true;
        }
    }
}

} // namespace optsvr
