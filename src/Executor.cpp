#include "Executor.h"
#include "RuntimeConfig.h"
#include "MapTuner.h"
#include "DHExecutor.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <list>
#include <string>
#include <sstream>

#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
namespace blog = boost::log;
extern blog::sources::severity_logger<blog::trivial::severity_level> lg;

// DHExecutor 执行器实例 (warehouse fullfill, EXECUTOR_MODE=3)
static optsvr::DHExecutor g_dh_executor;

void Executor::initialize(int preprocess_time_limit)
{
    tpg.resize(env->map.size());
    previous_locations.resize(env->num_of_agents, -1);
    official_process_initialized_ = false;

    max_counter_ = std::max(1, env->max_counter);

    int tmp = (env->action_time * env->max_counter);
    action_ceil_size = (env->min_planner_communication_time + tmp - 1) / tmp;
    stage_action_max = max(action_ceil_size * 2, 5);

    if (process_plan_type == APPEND_WINDOW) {
        window_size = env->min_planner_communication_time / (env->action_time * env->max_counter) + 1;
        if (window_size < 1) window_size = 1;
    } else if (process_plan_type == APPEND_ONE) {
        window_size = 1;
    } else {
        window_size = INT_MAX;
    }

    optsvr::load_map_tuning(env);

    // 初始化 DHExecutor（warehouse fullfill）
    int exec_mode = optsvr::cfg().executor_mode;
    if (const char* v = std::getenv("OPT_EXECUTOR_MODE")) exec_mode = std::atoi(v);
    if (exec_mode == 3) {
        g_dh_executor.initialize(this);
    }
}

bool Executor::is_valid_location(int location) const
{
    return location >= 0 &&
           location < (int)tpg.size() &&
           location < (int)env->map.size() &&
           env->map[location] == 0;
}

bool Executor::contains_agent_in_tpg(int location, int agent_id) const
{
    if (!is_valid_location(location)) return false;
    const auto& q = tpg[location];
    return std::find(q.begin(), q.end(), agent_id) != q.end();
}

bool Executor::erase_agent_from_tpg(int location, int agent_id)
{
    if (!is_valid_location(location)) return false;
    auto& q = tpg[location];
    auto it = std::find(q.begin(), q.end(), agent_id);
    if (it == q.end()) return false;
    q.erase(it);
    return true;
}

void Executor::rebuild_from_system_states()
{
    if ((int)previous_locations.size() != env->num_of_agents)
        previous_locations.assign(env->num_of_agents, -1);

    predicted_states = env->system_states;
    for (auto& q : tpg) q.clear();

    for (int i = 0; i < env->num_of_agents; i++) {
        const int loc = env->system_states[i].location;
        previous_locations[i] = loc;
        if (is_valid_location(loc)) tpg[loc].push_back(i);
    }
    official_process_initialized_ = true;
}

vector<State> Executor::process_new_plan_official(int sync_time_limit, Plan& plan_struct,
                                                 vector<vector<Action>>& staged_actions)
{
    if (predicted_states.size() != (size_t)env->num_of_agents)
        predicted_states = env->system_states;

    bool need_rebuild = !official_process_initialized_ ||
                        (int)previous_locations.size() != env->num_of_agents;
    if (env->system_timestep == 0) need_rebuild = true;
    if (need_rebuild) rebuild_from_system_states();

    const std::vector<std::vector<Action>>& plan = plan_struct.actions;
    if (plan.empty() || (int)plan.size() != env->num_of_agents)
        return predicted_states;

    int plan_horizon = INT_MAX;
    for (int i = 0; i < env->num_of_agents; i++)
        plan_horizon = std::min(plan_horizon, (int)plan[i].size());
    if (plan_horizon == INT_MAX) plan_horizon = 0;

    auto curr_states = predicted_states;
    int moves[4] = {1, env->cols, -1, -env->cols};
    const int append_timesteps = std::min(plan_horizon, window_size);

    for (int timestep = 0; timestep < append_timesteps; timestep++) {
        for (int i = 0; i < (int)plan.size(); i++) {
            Action action = plan[i][timestep];
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            bool executable_action = (action != Action::NA && action != Action::W);

            if (action == Action::FW) {
                const int candidate = new_location + moves[curr_states[i].orientation];
                if (is_valid_location(candidate)) {
                    new_location = candidate;
                    tpg[new_location].push_back(i);
                } else executable_action = false;
            } else if (action == Action::CR) {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            } else if (action == Action::CCR) {
                new_orientation = (curr_states[i].orientation + 3) % 4;
            }

            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;
            if (executable_action) staged_actions[i].push_back(action);
        }
    }

    vector<int> action_index(env->num_of_agents, 0);
    curr_states = env->system_states;
    std::vector<std::list<int>> temp_tpg(env->map.size());

    for (int t = 0; t < window_size; t++) {
        auto pre_states = curr_states;
        for (int i = 0; i < env->num_of_agents; i++) {
            if ((int)staged_actions[i].size() <= action_index[i]) {
                curr_states[i].timestep++;
                continue;
            }
            Action curr_action = staged_actions[i][action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location, new_orientation = curr_orientation;
            const bool at_tpg_front = is_valid_location(curr_location) &&
                !tpg[curr_location].empty() && tpg[curr_location].front() == i;

            if (curr_action == Action::FW) {
                const int candidate = curr_location + moves[curr_orientation];
                if (at_tpg_front && is_valid_location(candidate) &&
                    !tpg[candidate].empty() && tpg[candidate].front() == i) {
                    new_location = candidate; action_index[i]++;
                }
            } else if (curr_action == Action::CR) {
                new_orientation = (curr_orientation + 1) % 4;
                if (at_tpg_front) action_index[i]++;
            } else if (curr_action == Action::CCR) {
                new_orientation = (curr_orientation + 3) % 4;
                if (at_tpg_front) action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;
            curr_states[i].timestep += 1;
        }
        for (int i = 0; i < env->num_of_agents; i++) {
            const int prev_loc = pre_states[i].location, next_loc = curr_states[i].location;
            if (t == 0 && is_valid_location(prev_loc)) temp_tpg[prev_loc].push_back(i);
            if (prev_loc != next_loc) {
                if (is_valid_location(prev_loc) && !tpg[prev_loc].empty() && tpg[prev_loc].front() == i) tpg[prev_loc].pop_front();
                if (is_valid_location(next_loc)) temp_tpg[next_loc].push_back(i);
            }
        }
    }

    for (int i = 0; i < env->num_of_agents; i++) {
        if (action_index[i] < (int)staged_actions[i].size()) staged_actions[i].resize(action_index[i]);
        previous_locations[i] = env->system_states[i].location;
    }
    predicted_states = curr_states;
    tpg.swap(temp_tpg);
    official_process_initialized_ = true;

    return predicted_states;
}

// ════════════════════════════════════════════════════════════════════
// 分流入口
// ════════════════════════════════════════════════════════════════════
vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct,
                                         vector<vector<Action>>& staged_actions)
{
    switch (optsvr::cfg().executor_mode) {
        case 3:
            if (!g_dh_executor.is_initialized()) g_dh_executor.initialize(this);
            return g_dh_executor.process_new_plan(sync_time_limit, plan_struct, staged_actions);
        default: return process_new_plan_official(sync_time_limit, plan_struct, staged_actions);
    }
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command)
{
    // DHExecutor 维护自己的 TPG，跳过官方 TPG 更新
    if (optsvr::cfg().executor_mode == 3) {
        if (!g_dh_executor.is_initialized()) g_dh_executor.initialize(this);
        g_dh_executor.next_command(exec_time_limit, agent_command);
        return;
    }

    // 更新 TPG
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        if (prev_location != curr_location) {
            erase_agent_from_tpg(prev_location, i);
            previous_locations[i] = curr_location;
            if (is_valid_location(curr_location)) {
                if (!contains_agent_in_tpg(curr_location, i))
                    tpg[curr_location].push_front(i);
            }
        }
    }

    // Default: 基础 MCP
    for (int i = 0; i < (int)env->system_states.size(); i++)
        mcp_default_executor(i, agent_command);
}

bool Executor::mcp_default_executor(int agent_id, std::vector<ExecutionCommand>& agent_command)
{
    int moves[4] = {1, env->cols, -1, -env->cols};
    int curr_location = env->system_states[agent_id].location;
    int curr_orientation = env->system_states[agent_id].orientation;
    int next_location = curr_location;
    if (curr_orientation >= 0 && curr_orientation < 4)
        next_location = curr_location + moves[curr_orientation];

    if (env->staged_actions[agent_id].empty()) {
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    } else if (env->staged_actions[agent_id].front() != Action::FW) {
        // 旋转
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    }

    // 尝试前进
    if (!is_valid_location(next_location)) {
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    }

    const auto& next_queue = tpg[next_location];
    if (!is_valid_location(curr_location)) {
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    }
    const auto& curr_queue = tpg[curr_location];

    // 当前格不在队首时等待
    if (curr_queue.empty() || curr_queue.front() != agent_id) {
        agent_command[agent_id] = ExecutionCommand::STOP; return false;
    }

    // 目标格无 agent 或自己在队首则前进
    if (next_queue.empty() || next_queue.front() == agent_id) {
        agent_command[agent_id] = ExecutionCommand::GO;
        return true;
    }

    agent_command[agent_id] = ExecutionCommand::STOP;
    return false;
}
