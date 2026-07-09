// optsvr/executor/RoomOpt/RoomOptExecutor.cpp
// 来源: IDEA/RoomOpt/Executor.cpp (v10.25)
// 连续 STOP 截断：追踪每个 agent 连续被 STOP 的 tick 数，
// 达到 TRUNCATE_THRESHOLD(=2) 时清空 staged_actions 触发重规划。
// 避免 v10.20 在首次 STOP 就截断导致 avg 过低的问题。

#include "RoomOptExecutor.h"
#include <algorithm>
#include <cstdlib>

namespace optsvr {

void RoomOptExecutor::initialize(Executor* exec) {
    exec_ = exec;
    env_ = exec->env;

    window_size_ = env_->min_planner_communication_time / (env_->action_time * env_->max_counter) + 1;
    if (window_size_ < 1) window_size_ = 1;

    previous_locations_.resize(env_->num_of_agents);
    consecutive_stop_.assign(env_->num_of_agents, 0);

    // 环境变量覆盖截断阈值
    if (const char* v = std::getenv("OPT_ROOM_TRUNC_THRESH"))
        truncate_threshold_ = std::max(1, std::atoi(v));

    first_execution_ = true;
}

std::vector<State> RoomOptExecutor::process_new_plan(
    int sync_time_limit, Plan& plan_struct,
    std::vector<std::vector<Action>>& staged_actions) {

    if ((int)predicted_states_.size() != env_->num_of_agents)
        predicted_states_ = env_->system_states;

    if (first_execution_) {
        tpg_.clear();
        for (int i = 0; i < env_->num_of_agents; i++) {
            tpg_[env_->system_states[i].location].push_back(i);
            previous_locations_[i] = env_->system_states[i].location;
        }
        first_execution_ = false;
    }

    for (int i = 0; i < env_->num_of_agents; i++)
        previous_locations_[i] = env_->system_states[i].location;

    auto curr_states = predicted_states_;
    int moves[4] = {1, env_->cols, -1, -env_->cols};
    const auto& plan = plan_struct.actions;

    // Phase 1: append plan to staged_actions + TPG
    for (int timestep = 0; timestep < (int)plan[0].size() && timestep < window_size_; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            if (plan[i][timestep] == Action::FW) {
                new_location += moves[curr_states[i].orientation];
                tpg_[new_location].push_back(i);
            } else if (plan[i][timestep] == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (plan[i][timestep] == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;
            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
                staged_actions[i].push_back(plan[i][timestep]);
        }
    }

    // Phase 2: simulate execution to truncate
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
            int new_location = curr_location;
            int new_orientation = curr_states[i].orientation;

            if (tpg_[curr_location].empty() || tpg_[curr_location].front() != i) {
                curr_states[i].timestep++;
                continue;
            }

            if (curr_action == Action::FW) {
                new_location = curr_location + moves[curr_states[i].orientation];
                if (!tpg_[new_location].empty() && tpg_[new_location].front() != i)
                    new_location = curr_location;
                else
                    action_index[i]++;
            } else if (curr_action == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
                action_index[i]++;
            } else if (curr_action == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
                action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
        }

        for (int i = 0; i < env_->num_of_agents; i++) {
            if (t == 0) temp_tpg_[pre_states[i].location].push_back(i);
            if (pre_states[i].location != curr_states[i].location) {
                auto& ol = tpg_[pre_states[i].location];
                if (!ol.empty() && ol.front() == i) ol.pop_front();
                temp_tpg_[curr_states[i].location].push_back(i);
            }
        }
    }

    // Truncate unreachable actions
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (action_index[i] < (int)staged_actions[i].size())
            staged_actions[i].resize(action_index[i]);
    }

    predicted_states_ = curr_states;
    tpg_ = temp_tpg_;

    // v8.50: reset predicted_states for 0-action agents
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (action_index[i] == 0) {
            curr_states[i] = env_->system_states[i];
            curr_states[i].timestep = env_->system_states[i].timestep;
        }
    }
    predicted_states_ = curr_states;

    // Reset consecutive_stop on new plan
    consecutive_stop_.assign(env_->num_of_agents, 0);

    return predicted_states_;
}

void RoomOptExecutor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command) {
    int moves[4] = {1, env_->cols, -1, -env_->cols};

    // Update previous_locations in TPG
    for (int i = 0; i < env_->num_of_agents; i++) {
        int prev_location = previous_locations_[i];
        int curr_location = env_->system_states[i].location;
        if (prev_location != curr_location) {
            auto& ol = tpg_[prev_location];
            if (!ol.empty() && ol.front() == i) ol.pop_front();
            previous_locations_[i] = curr_location;
        }
    }

    // GO/STOP decision with consecutive-STOP truncation
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (env_->staged_actions[i].empty()) {
            agent_command[i] = ExecutionCommand::STOP;
            consecutive_stop_[i] = 0;
            continue;
        }

        if (env_->staged_actions[i].front() != Action::FW) {
            agent_command[i] = ExecutionCommand::GO;
            consecutive_stop_[i] = 0;
            continue;
        }

        int curr_location = env_->system_states[i].location;
        int curr_orientation = env_->system_states[i].orientation;
        int next_location = curr_location + moves[curr_orientation];

        if (tpg_[next_location].empty() || tpg_[next_location].front() == i) {
            agent_command[i] = ExecutionCommand::GO;
            consecutive_stop_[i] = 0;
        } else {
            agent_command[i] = ExecutionCommand::STOP;
            consecutive_stop_[i]++;
            // v10.25: Only truncate after threshold consecutive STOP ticks
            if (consecutive_stop_[i] >= truncate_threshold_) {
                env_->staged_actions[i].clear();
                consecutive_stop_[i] = 0;
            }
        }
    }
}

} // namespace optsvr
