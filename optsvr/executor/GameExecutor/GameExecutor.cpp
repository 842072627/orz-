// optsvr/executor/GameExecutor/GameExecutor.cpp
// 来源: 基于 DHExecutor(mode=3) 全量继承 + RoomOptExecutor(mode=4) 截断策略
// EXECUTOR_MODE=6 时使用
#include "GameExecutor.h"
#include "RuntimeConfig.h"
#include <algorithm>
#include <cassert>
#include <cstdlib>

namespace optsvr {

namespace {

template <typename QueueMap>
void erase_from_tpg(QueueMap& queues, int location, int agent_id)
{
    auto it = queues.find(location);
    if (it == queues.end()) return;
    auto& q = it->second;
    auto qit = std::find(q.begin(), q.end(), agent_id);
    if (qit != q.end()) q.erase(qit);
}

} // namespace

// ==================== 辅助函数 (与 DHExecutor 完全一致) ====================

int GameExecutor::get_next_location(const State& state, SharedEnvironment* env) {
    int moves[4] = {1, env->cols, -1, -env->cols};
    return state.location + moves[state.orientation];
}

int GameExecutor::planned_actions_to_location(
    int agent_id, int target_location,
    const std::vector<State>& states,
    const std::vector<std::vector<Action>>& staged_actions,
    const std::vector<int>& action_index,
    SharedEnvironment* env)
{
    if (states[agent_id].location == target_location) return 0;

    int moves[4] = {1, env->cols, -1, -env->cols};
    int location = states[agent_id].location;
    int orientation = states[agent_id].orientation;
    int actions_to_target = 0;

    for (int k = action_index[agent_id]; k < (int)staged_actions[agent_id].size(); k++) {
        Action action = staged_actions[agent_id][k];
        actions_to_target++;
        if (action == Action::FW) {
            location += moves[orientation];
            if (location == target_location) return actions_to_target;
        } else if (action == Action::CR) {
            orientation = (orientation + 1) % 4;
        } else if (action == Action::CCR) {
            orientation = (orientation - 1) % 4;
            if (orientation == -1) orientation = 3;
        }
    }
    return TEMPORARY_PASS_NOT_REACHED;
}

int GameExecutor::planned_actions_to_leave_location_after_entry(
    int agent_id, int entered_location,
    const std::vector<State>& states,
    const std::vector<std::vector<Action>>& staged_actions,
    const std::vector<int>& action_index,
    SharedEnvironment* env)
{
    int curr_action_index = action_index[agent_id];
    if (curr_action_index >= (int)staged_actions[agent_id].size() ||
        staged_actions[agent_id][curr_action_index] != Action::FW ||
        get_next_location(states[agent_id], env) != entered_location)
    {
        return TEMPORARY_PASS_NOT_REACHED;
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    int location = entered_location;
    int orientation = states[agent_id].orientation;
    int actions_after_entry = 0;

    for (int k = curr_action_index + 1; k < (int)staged_actions[agent_id].size(); k++) {
        Action action = staged_actions[agent_id][k];
        actions_after_entry++;
        if (action == Action::FW) {
            location += moves[orientation];
            if (location != entered_location) return actions_after_entry;
        } else if (action == Action::CR) {
            orientation = (orientation + 1) % 4;
        } else if (action == Action::CCR) {
            orientation = (orientation - 1) % 4;
            if (orientation == -1) orientation = 3;
        }
    }
    return TEMPORARY_PASS_NOT_REACHED;
}

bool GameExecutor::location_is_currently_empty(
    int location, int agent_id, const std::vector<State>& states)
{
    for (int i = 0; i < (int)states.size(); i++) {
        if (i != agent_id && states[i].location == location) return false;
    }
    return true;
}

bool GameExecutor::promote_temporary_pass_if_safe(
    int location, int agent_id,
    const std::vector<State>& states,
    const std::vector<std::vector<Action>>& staged_actions,
    const std::vector<int>& action_index)
{
    auto tpg_it = tpg_.find(location);
    if (tpg_it == tpg_.end() || tpg_it->second.empty()) return false;

    auto& visit_order = tpg_it->second;
    auto agent_visit = visit_order.end();
    for (auto it = visit_order.begin(); it != visit_order.end(); ++it) {
        if (*it == agent_id) { agent_visit = it; break; }
    }
    if (agent_visit == visit_order.end() || agent_visit == visit_order.begin()) return false;

    if (!location_is_currently_empty(location, agent_id, states)) return false;

    int actions_to_leave = planned_actions_to_leave_location_after_entry(
        agent_id, location, states, staged_actions, action_index, env_);
    if (actions_to_leave == TEMPORARY_PASS_NOT_REACHED) return false;

    for (auto it = visit_order.begin(); it != agent_visit; ++it) {
        int blocking_agent = *it;
        int actions_away = planned_actions_to_location(
            blocking_agent, location, states, staged_actions, action_index, env_);
        if (actions_away < TEMPORARY_PASS_MIN_ACTIONS_AWAY ||
            actions_to_leave > actions_away)
        {
            return false;
        }
    }

    visit_order.erase(agent_visit);
    visit_order.push_front(agent_id);
    return true;
}

// ==================== 公共接口 ====================

void GameExecutor::initialize(Executor* exec) {
    exec_ = exec;
    env_ = exec->env;
    window_size_ = (int)env_->min_planner_communication_time /
                   (env_->action_time * env_->max_counter) + 1;
    previous_locations_.resize(env_->num_of_agents, -1);
    consecutive_stop_.assign(env_->num_of_agents, 0);
    first_execution_ = true;
    tpg_.clear();
    temp_tpg_.clear();
    predicted_states_.clear();

    // 环境变量覆盖开关
    if (const char* v = std::getenv("GAME_EXEC_TRUNCATE"))
        enable_truncate_ = (std::string(v) == "1" || std::string(v) == "true");
    if (const char* v = std::getenv("GAME_EXEC_TRUNC_THRESH"))
        truncate_threshold_ = std::max(1, std::atoi(v));
}

std::vector<State> GameExecutor::process_new_plan(
    int sync_time_limit, Plan& plan_struct,
    std::vector<std::vector<Action>>& staged_actions)
{
    if ((int)predicted_states_.size() != env_->num_of_agents) {
        predicted_states_ = env_->system_states;
    }

    if (first_execution_) {
        for (int i = 0; i < env_->num_of_agents; i++) {
            tpg_[env_->curr_states[i].location].push_back(i);
        }
        first_execution_ = false;
    }

    for (int i = 0; i < env_->num_of_agents; i++) {
        previous_locations_[i] = env_->system_states[i].location;
    }

    auto curr_states = predicted_states_;
    int moves[4] = {1, env_->cols, -1, -env_->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

    // Phase 1: append plan to staged_actions + TPG (与 DHExecutor 完全一致)
    for (int timestep = 0; timestep < (int)plan[0].size() && timestep < window_size_; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;

            if (plan[i][timestep] == Action::FW) {
                new_location = new_location + moves[curr_states[i].orientation];
                tpg_[new_location].push_back(i);
            } else if (plan[i][timestep] == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (plan[i][timestep] == Action::CCR) {
                new_orientation = (curr_states[i].orientation - 1) % 4;
                if (new_orientation == -1) new_orientation = 3;
            }

            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;

            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W) {
                staged_actions[i].push_back(plan[i][timestep]);
            }
        }
    }

    // Phase 2: MCP simulation with promote_temporary_pass_if_safe (与 DHExecutor 完全一致)
    std::vector<int> action_index(env_->num_of_agents, 0);
    curr_states = env_->system_states;
    temp_tpg_.clear();

    for (int t = 0; t < window_size_; t++) {
        auto pre_states = curr_states;
        for (int i = 0; i < env_->num_of_agents; i++) {
            if ((int)staged_actions[i].size() <= action_index[i]) {
                curr_states[i].timestep++;
                continue;
            }

            Action curr_action = staged_actions[i][action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location;
            int new_orientation = curr_orientation;

            auto tpg_curr_it = tpg_.find(curr_location);
            bool at_front = tpg_curr_it != tpg_.end() &&
                            !tpg_curr_it->second.empty() &&
                            tpg_curr_it->second.front() == i;
            if (!at_front) {
                curr_states[i].timestep++;
                continue;
            }

            if (curr_action == Action::FW) {
                new_location = curr_location + moves[curr_orientation];
                auto tpg_next_it = tpg_.find(new_location);
                if (tpg_next_it == tpg_.end() || tpg_next_it->second.empty()) {
                    new_location = curr_location;
                } else if (tpg_next_it->second.front() == i ||
                    promote_temporary_pass_if_safe(new_location, i, curr_states, staged_actions, action_index))
                {
                    action_index[i]++;
                } else {
                    new_location = curr_location;
                }
            } else if (curr_action == Action::CR) {
                new_orientation = (curr_orientation + 1) % 4;
                action_index[i]++;
            } else if (curr_action == Action::CCR) {
                new_orientation = (curr_orientation - 1) % 4;
                if (new_orientation == -1) new_orientation = 3;
                action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
        }
        // Clear dependency
        for (int i = 0; i < env_->num_of_agents; i++) {
            if (t == 0) {
                temp_tpg_[pre_states[i].location].push_back(i);
            }
            if (pre_states[i].location != curr_states[i].location) {
                erase_from_tpg(tpg_, pre_states[i].location, i);
                temp_tpg_[curr_states[i].location].push_back(i);
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

    // Reset consecutive_stop on new plan
    consecutive_stop_.assign(env_->num_of_agents, 0);

    return predicted_states_;
}

void GameExecutor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command) {
    int moves[4] = {1, env_->cols, -1, -env_->cols};

    // Update TPG based on actual movement (与 DHExecutor 完全一致)
    for (int i = 0; i < (int)env_->system_states.size(); i++) {
        int prev_location = previous_locations_[i];
        int curr_location = env_->system_states[i].location;
        if (prev_location != curr_location) {
            erase_from_tpg(tpg_, prev_location, i);
            previous_locations_[i] = curr_location;
        }
    }

    // MCP 决策
    for (int i = 0; i < (int)env_->system_states.size(); i++) {
        mcp(i, agent_command);
    }
}

void GameExecutor::mcp(int agent_id, std::vector<ExecutionCommand>& agent_command) {
    if (env_->staged_actions[agent_id].empty()) {
        agent_command[agent_id] = ExecutionCommand::STOP;
        consecutive_stop_[agent_id] = 0;
        return;
    }

    if (env_->staged_actions[agent_id].front() != Action::FW) {
        agent_command[agent_id] = ExecutionCommand::GO;
        consecutive_stop_[agent_id] = 0;
        return;
    }

    int moves[4] = {1, env_->cols, -1, -env_->cols};
    int curr_location = env_->system_states[agent_id].location;
    int curr_orientation = env_->system_states[agent_id].orientation;
    int next_location = curr_location + moves[curr_orientation];

    auto tpg_it = tpg_.find(next_location);
    if (tpg_it == tpg_.end() || tpg_it->second.empty() || tpg_it->second.front() == agent_id) {
        agent_command[agent_id] = ExecutionCommand::GO;
        consecutive_stop_[agent_id] = 0;
    } else {
        agent_command[agent_id] = ExecutionCommand::STOP;
        consecutive_stop_[agent_id]++;

        // 连续 STOP 截断 (来自 RoomOptExecutor)
        if (enable_truncate_ && consecutive_stop_[agent_id] >= truncate_threshold_) {
            env_->staged_actions[agent_id].clear();
            consecutive_stop_[agent_id] = 0;
        }
    }
}

} // namespace optsvr
