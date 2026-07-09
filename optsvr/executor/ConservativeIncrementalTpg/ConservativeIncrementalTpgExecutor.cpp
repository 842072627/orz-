// optsvr/executor/ConservativeIncrementalTpg/ConservativeIncrementalTpgExecutor.cpp
// Mode 8: preserves the conservative IncrementalTpgExecutor semantics from
// commit 9f9bf33, including old/new staged-action distinction.
#include "ConservativeIncrementalTpgExecutor.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <climits>

namespace optsvr {

namespace {

bool staged_action(Action action)
{
    return action != Action::W && action != Action::NA;
}

bool tpg_allows_entry(
    const std::vector<std::list<int>>& queues,
    int location,
    int agent_id,
    const Executor* exec)
{
    if (exec == nullptr ||
        !exec->is_valid_location(location) ||
        location >= (int)queues.size())
        return false;

    const auto& q = queues[location];
    if (q.empty()) return true;
    return q.front() == agent_id;
}

void erase_from_queue(std::list<int>& q, int agent_id)
{
    auto it = std::find(q.begin(), q.end(), agent_id);
    if (it != q.end()) q.erase(it);
}

bool queue_contains(const std::list<int>& q, int agent_id)
{
    return std::find(q.begin(), q.end(), agent_id) != q.end();
}

} // namespace

void ConservativeIncrementalTpgExecutor::initialize(Executor* exec)
{
    exec_ = exec;
    env_ = exec != nullptr ? exec->env : nullptr;
}

bool ConservativeIncrementalTpgExecutor::is_tpg_front(
    const std::vector<std::list<int>>& queues,
    int location,
    int agent_id) const
{
    return exec_ != nullptr &&
           exec_->is_valid_location(location) &&
           location < (int)queues.size() &&
           !queues[location].empty() &&
           queues[location].front() == agent_id;
}

std::vector<State> ConservativeIncrementalTpgExecutor::process_new_plan(
    int /*sync_time_limit*/,
    Plan& plan_struct,
    std::vector<std::vector<Action>>& staged_actions)
{
    if (exec_ == nullptr || env_ == nullptr) return {};

    const int n = env_->num_of_agents;
    if ((int)exec_->predicted_states.size() != n)
        exec_->predicted_states = env_->system_states;

    bool need_rebuild = !exec_->official_process_initialized_ ||
                        (int)exec_->previous_locations.size() != n;
    if (env_->system_timestep == 0) need_rebuild = true;
    if (need_rebuild) exec_->rebuild_from_system_states();

    const std::vector<std::vector<Action>>& plan = plan_struct.actions;
    if (plan.empty() || (int)plan.size() != n)
        return exec_->predicted_states;

    int plan_horizon = INT_MAX;
    for (int i = 0; i < n; i++)
        plan_horizon = std::min(plan_horizon, (int)plan[i].size());
    if (plan_horizon == INT_MAX) plan_horizon = 0;

    const int append_timesteps = std::min(plan_horizon, exec_->window_size);
    int moves[4] = {1, env_->cols, -1, -env_->cols};

    std::vector<int> old_len(n, 0);
    std::vector<std::vector<Action>> candidates(n);
    for (int i = 0; i < n; i++) {
        old_len[i] = (int)staged_actions[i].size();
        candidates[i].reserve(staged_actions[i].size() + append_timesteps);
        for (Action action : staged_actions[i])
            candidates[i].push_back(action);
    }

    std::vector<State> append_states = exec_->predicted_states;
    for (int timestep = 0; timestep < append_timesteps; timestep++) {
        for (int i = 0; i < n; i++) {
            Action action = plan[i][timestep];
            int new_location = append_states[i].location;
            int new_orientation = append_states[i].orientation;
            bool executable_action = staged_action(action);

            if (action == Action::FW) {
                int candidate = -1;
                if (new_orientation >= 0 && new_orientation < 4)
                    candidate = new_location + moves[new_orientation];
                if (exec_->is_valid_location(candidate)) {
                    new_location = candidate;
                } else {
                    executable_action = false;
                }
            } else if (action == Action::CR) {
                new_orientation = (new_orientation + 1) % 4;
            } else if (action == Action::CCR) {
                new_orientation = (new_orientation + 3) % 4;
            }

            append_states[i].location = new_location;
            append_states[i].orientation = new_orientation;
            append_states[i].timestep += 1;
            if (executable_action) candidates[i].push_back(action);
        }
    }

    int sim_steps = exec_->window_size;
    if (sim_steps == INT_MAX) {
        sim_steps = 0;
        for (int i = 0; i < n; i++)
            sim_steps = std::max(sim_steps, (int)candidates[i].size());
    }

    std::vector<std::vector<Action>> accepted(n);
    std::vector<int> action_index(n, 0);
    for (int i = 0; i < n; i++)
        accepted[i].reserve(std::min(sim_steps, (int)candidates[i].size()));

    std::vector<std::list<int>> sim_tpg = exec_->tpg;
    if ((int)sim_tpg.size() != (int)env_->map.size())
        sim_tpg.resize(env_->map.size());
    std::vector<std::list<int>> final_tpg(env_->map.size());
    for (int i = 0; i < n; i++) {
        const int loc = env_->system_states[i].location;
        if (exec_->is_valid_location(loc))
            final_tpg[loc].push_back(i);
    }
    std::vector<State> sim_states = env_->system_states;

    for (int t = 0; t < sim_steps; t++) {
        std::vector<int> prev_locations(n, -1);
        std::vector<int> next_locations(n, -1);

        for (int i = 0; i < n; i++) {
            prev_locations[i] = sim_states[i].location;

            if (action_index[i] >= (int)candidates[i].size()) {
                sim_states[i].timestep += 1;
                next_locations[i] = sim_states[i].location;
                continue;
            }

            const int curr_action_index = action_index[i];
            const bool old_action = curr_action_index < old_len[i];
            const Action action = candidates[i][curr_action_index];
            const int curr_location = sim_states[i].location;
            const int curr_orientation = sim_states[i].orientation;

            if (!exec_->is_valid_location(curr_location) ||
                !queue_contains(sim_tpg[curr_location], i) ||
                !tpg_allows_entry(sim_tpg, curr_location, i, exec_)) {
                sim_states[i].timestep += 1;
                next_locations[i] = sim_states[i].location;
                continue;
            }

            if (action == Action::FW) {
                int candidate = -1;
                if (curr_orientation >= 0 && curr_orientation < 4)
                    candidate = curr_location + moves[curr_orientation];

                bool can_advance = false;
                if (exec_->is_valid_location(candidate)) {
                    if (old_action) {
                        can_advance = !sim_tpg[candidate].empty() &&
                                      tpg_allows_entry(sim_tpg, candidate, i, exec_);
                    } else {
                        can_advance = tpg_allows_entry(sim_tpg, candidate, i, exec_);
                    }
                }

                if (can_advance) {
                    accepted[i].push_back(action);
                    action_index[i]++;
                    sim_states[i].location = candidate;
                    if (!old_action && sim_tpg[candidate].empty())
                        sim_tpg[candidate].push_back(i);
                    final_tpg[candidate].push_back(i);
                }
            } else if (action == Action::CR) {
                accepted[i].push_back(action);
                action_index[i]++;
                sim_states[i].orientation = (curr_orientation + 1) % 4;
            } else if (action == Action::CCR) {
                accepted[i].push_back(action);
                action_index[i]++;
                sim_states[i].orientation = (curr_orientation + 3) % 4;
            }

            sim_states[i].timestep += 1;
            next_locations[i] = sim_states[i].location;
        }

        for (int i = 0; i < n; i++) {
            const int prev = prev_locations[i];
            const int next = next_locations[i];
            if (prev == next) continue;
            if (!exec_->is_valid_location(prev)) continue;

            if (!sim_tpg[prev].empty() && sim_tpg[prev].front() == i)
                sim_tpg[prev].pop_front();
            else
                erase_from_queue(sim_tpg[prev], i);
        }
    }

    staged_actions.swap(accepted);
    exec_->predicted_states = sim_states;
    exec_->tpg.swap(final_tpg);

    if ((int)exec_->previous_locations.size() != n)
        exec_->previous_locations.assign(n, -1);
    for (int i = 0; i < n; i++)
        exec_->previous_locations[i] = env_->system_states[i].location;

    exec_->official_process_initialized_ = true;
    return exec_->predicted_states;
}

void ConservativeIncrementalTpgExecutor::next_command(
    int /*exec_time_limit*/,
    std::vector<ExecutionCommand>& agent_command)
{
    if (exec_ == nullptr || env_ == nullptr) return;
    for (int i = 0; i < (int)env_->system_states.size(); i++)
        exec_->mcp_default_executor(i, agent_command);
}

} // namespace optsvr
