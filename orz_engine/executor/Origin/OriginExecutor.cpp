// optsvr/executor/Origin/OriginExecutor.cpp
// Origin 模式实现：自适应 append + MCP 跟随链
#include "OriginExecutor.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <functional>
#include <unordered_map>

namespace orz_engine {

void OriginExecutor::initialize(Executor* exec) {
    exec_ = exec;
    env_ = exec->env;
    turn_ = 0;
    total_push_action_count_ = 0;
    following_chain_go_count_ = 0;
}

std::vector<State> OriginExecutor::process_new_plan(
    int /*sync_time_limit*/, Plan& plan_struct,
    std::vector<std::vector<Action>>& staged_actions)
{
    turn_++;

    if (exec_->predicted_states.size() != (size_t)env_->num_of_agents)
        exec_->predicted_states = env_->system_states;

    if (env_->system_timestep == 0)
        exec_->rebuild_from_system_states();

    auto curr_states = exec_->predicted_states;
    const std::vector<std::vector<Action>>& plan = plan_struct.actions;
    if (plan.empty() || (int)plan.size() != env_->num_of_agents)
        return exec_->predicted_states;

    int plan_horizon = INT_MAX;
    for (int i = 0; i < env_->num_of_agents; i++)
        plan_horizon = std::min(plan_horizon, (int)plan[i].size());
    if (plan_horizon == INT_MAX) plan_horizon = 0;

    int moves[4] = {1, env_->cols, -1, -env_->cols};

    const auto& c = cfg();
    int action_windows = c.origin_append_steps;
    if (action_windows == 0) {
        action_windows = exec_->window_size;
        if ((action_windows - 1) * env_->max_counter * env_->action_time + total_push_action_count_ >=
            turn_ * env_->min_planner_communication_time) {
            action_windows--;
        }
        total_push_action_count_ += action_windows * env_->max_counter * env_->action_time;
    }

    int total_ts = std::min(plan_horizon, action_windows);
    const int n = env_->num_of_agents;
    std::vector<int> agent_ts(n, total_ts);

    // 逐 timestep append
    std::vector<int> added(n, 0);
    for (int timestep = 0; timestep < total_ts; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            if (added[i] >= agent_ts[i]) {
                exec_->predicted_states[i].timestep += 1;
                curr_states[i] = exec_->predicted_states[i];
                continue;
            }
            Action action = plan[i][timestep];
            bool executable_action = (action != Action::NA && action != Action::W);
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            if (action == Action::FW) {
                const int candidate = new_location + moves[curr_states[i].orientation];
                if (exec_->is_valid_location(candidate)) {
                    new_location = candidate;
                    exec_->tpg[new_location].push_back(i);
                }
            } else if (action == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (action == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
            }
            exec_->predicted_states[i].location = new_location;
            exec_->predicted_states[i].orientation = new_orientation;
            exec_->predicted_states[i].timestep += 1;
            curr_states[i] = exec_->predicted_states[i];
            added[i]++;
            if (executable_action) staged_actions[i].push_back(action);
        }
    }

    return exec_->predicted_states;
}

void OriginExecutor::next_command(int /*exec_time_limit*/,
                                  std::vector<ExecutionCommand>& agent_command)
{
    std::vector<bool> decided(env_->num_of_agents, false);
    for (int i = 0; i < (int)env_->system_states.size(); i++) {
        if (!decided[i]) mcp(i, decided, agent_command);
    }
}

bool OriginExecutor::mcp(int agent_id, std::vector<bool>& curr_decision,
                         std::vector<ExecutionCommand>& agent_command)
{
    if (env_->staged_actions[agent_id].empty()) {
        agent_command[agent_id] = ExecutionCommand::STOP;
        curr_decision[agent_id] = true;
        return false;
    } else if (env_->staged_actions[agent_id].front() != Action::FW) {
        agent_command[agent_id] = ExecutionCommand::GO;
        curr_decision[agent_id] = true;
        return false;
    } else {
        int moves[4] = {1, env_->cols, -1, -env_->cols};
        int curr_location = env_->system_states[agent_id].location;
        int curr_orientation = env_->system_states[agent_id].orientation;
        int next_location = curr_location + moves[curr_orientation];

        if (!exec_->is_valid_location(next_location)) {
            agent_command[agent_id] = ExecutionCommand::STOP;
            curr_decision[agent_id] = true;
            return false;
        }

        if (exec_->tpg[next_location].empty() || exec_->tpg[next_location].front() == agent_id) {
            agent_command[agent_id] = ExecutionCommand::GO;
            curr_decision[agent_id] = true;
            return true;
        }

        // 跟随链判定
        {
            const int blocker = exec_->tpg[next_location].front();
            const auto& blocker_state = env_->system_states[blocker];
            const auto& agent_state = env_->system_states[agent_id];

            if (blocker_state.location == next_location &&
                blocker_state.orientation == curr_orientation &&
                !env_->staged_actions[blocker].empty() &&
                env_->staged_actions[blocker].front() == Action::FW &&
                exec_->tpg[next_location].size() >= 2) {
                auto it = exec_->tpg[next_location].begin();
                ++it;
                if (it != exec_->tpg[next_location].end() && *it == agent_id) {
                    if (!curr_decision[blocker]) mcp(blocker, curr_decision, agent_command);
                    if (agent_state.counter.count < blocker_state.counter.count +
                        (agent_command[blocker] == ExecutionCommand::GO)) {
                        agent_command[agent_id] = ExecutionCommand::GO;
                        curr_decision[agent_id] = true;
                        following_chain_go_count_++;
                        return true;
                    }
                }
            }
        }

        agent_command[agent_id] = ExecutionCommand::STOP;
        curr_decision[agent_id] = true;
        return false;
    }
}

} // namespace orz_engine
