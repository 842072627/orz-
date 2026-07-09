// optsvr/executor/IncrementalTpg/IncrementalTpgExecutor.cpp
// Incremental TPG executor: accepts and registers only actions that pass the
// current simulated TPG state, avoiding phantom future reservations.
#include "IncrementalTpgExecutor.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <climits>
#include <unordered_map>
#include <utility>

namespace optsvr {

namespace {

bool staged_action(Action action)
{
    return action != Action::W && action != Action::NA;
}

void erase_from_queue(std::list<int>& q, int agent_id)
{
    auto it = std::find(q.begin(), q.end(), agent_id);
    if (it != q.end()) q.erase(it);
}

using SparseTpg = std::unordered_map<int, std::list<int>>;

bool sparse_tpg_empty(const SparseTpg& queues, int location)
{
    auto it = queues.find(location);
    return it == queues.end() || it->second.empty();
}

bool sparse_tpg_contains(const SparseTpg& queues, int location, int agent_id)
{
    auto it = queues.find(location);
    if (it == queues.end()) return false;
    const auto& q = it->second;
    return std::find(q.begin(), q.end(), agent_id) != q.end();
}

bool sparse_tpg_allows_entry(
    const SparseTpg& queues,
    int location,
    int agent_id)
{
    auto it = queues.find(location);
    if (it == queues.end() || it->second.empty()) return true;
    return it->second.front() == agent_id;
}

void sparse_tpg_push_back(SparseTpg& queues, int location, int agent_id)
{
    queues[location].push_back(agent_id);
}

void sparse_tpg_remove(SparseTpg& queues, int location, int agent_id)
{
    auto it = queues.find(location);
    if (it == queues.end()) return;
    auto& q = it->second;
    if (!q.empty() && q.front() == agent_id) {
        q.pop_front();
    } else {
        erase_from_queue(q, agent_id);
    }
    if (q.empty()) queues.erase(it);
}

} // namespace

void IncrementalTpgExecutor::initialize(Executor* exec)
{
    exec_ = exec;
    env_ = exec != nullptr ? exec->env : nullptr;
}

std::vector<State> IncrementalTpgExecutor::process_new_plan(
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

    const int candidate_limit = exec_->window_size == INT_MAX
        ? INT_MAX
        : std::max(0, exec_->window_size);
    std::vector<std::vector<Action>> candidates(n);
    for (int i = 0; i < n; i++) {
        const size_t reserve_size = candidate_limit == INT_MAX
            ? staged_actions[i].size() + append_timesteps
            : std::min<size_t>(candidate_limit, staged_actions[i].size() + append_timesteps);
        candidates[i].reserve(reserve_size);
        for (Action action : staged_actions[i]) {
            if (candidate_limit != INT_MAX &&
                (int)candidates[i].size() >= candidate_limit)
                break;
            candidates[i].push_back(action);
        }
    }

    // Match the official executor's append semantics: new planner actions are
    // interpreted from the previously predicted tail state, not directly from
    // the current physical state. The later crop simulation still starts from
    // env_->system_states.
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
            if (executable_action &&
                (candidate_limit == INT_MAX ||
                 (int)candidates[i].size() < candidate_limit))
                candidates[i].push_back(action);
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

    std::vector<std::pair<int, int>> final_reservations;
    const size_t reservation_cap =
        static_cast<size_t>(n) * static_cast<size_t>(std::max(1, sim_steps) + 2);
    SparseTpg sim_tpg;
    sim_tpg.reserve(reservation_cap);
    final_reservations.reserve(reservation_cap);
    for (int i = 0; i < n; i++) {
        const int loc = env_->system_states[i].location;
        if (exec_->is_valid_location(loc)) {
            sparse_tpg_push_back(sim_tpg, loc, i);
            final_reservations.emplace_back(loc, i);
        }
    }

    std::vector<State> sim_states = env_->system_states;

    // Reserve targets of forward actions that are already physically in
    // progress. Future-only reservations are intentionally rebuilt by the
    // incremental simulation below, so they cannot block unrelated agents
    // before the simulated blocker actually reaches the cell.
    std::vector<int> in_progress_fw_target(n, -1);
    for (int i = 0; i < n; i++) {
        const State& s = env_->system_states[i];
        if (s.current_action != Action::FW ||
            s.counter.count <= 0 ||
            s.orientation < 0 ||
            s.orientation >= 4)
            continue;

        const int target = s.location + moves[s.orientation];
        if (!exec_->is_valid_location(target)) continue;
        in_progress_fw_target[i] = target;
        sparse_tpg_push_back(sim_tpg, target, i);
        final_reservations.emplace_back(target, i);
    }

    std::vector<int> prev_locations(n, -1);
    std::vector<int> next_locations(n, -1);

    for (int t = 0; t < sim_steps; t++) {
        for (int i = 0; i < n; i++) {
            prev_locations[i] = sim_states[i].location;

            if (action_index[i] >= (int)candidates[i].size()) {
                sim_states[i].timestep += 1;
                next_locations[i] = sim_states[i].location;
                continue;
            }

            const Action action = candidates[i][action_index[i]];
            const int curr_location = sim_states[i].location;
            const int curr_orientation = sim_states[i].orientation;

            if (!sparse_tpg_contains(sim_tpg, curr_location, i) ||
                !sparse_tpg_allows_entry(sim_tpg, curr_location, i)) {
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
                    can_advance = sparse_tpg_allows_entry(sim_tpg, candidate, i);
                }

                if (can_advance) {
                    accepted[i].push_back(action);
                    action_index[i]++;
                    sim_states[i].location = candidate;
                    const bool already_reserved_in_progress =
                        in_progress_fw_target[i] == candidate &&
                        action_index[i] == 1;
                    if (sparse_tpg_empty(sim_tpg, candidate))
                        sparse_tpg_push_back(sim_tpg, candidate, i);
                    if (!already_reserved_in_progress)
                        final_reservations.emplace_back(candidate, i);
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

            sparse_tpg_remove(sim_tpg, prev, i);
        }
    }

    staged_actions.swap(accepted);
    exec_->predicted_states = sim_states;
    if ((int)exec_->tpg.size() != (int)env_->map.size())
        exec_->tpg.resize(env_->map.size());
    for (auto& q : exec_->tpg) q.clear();
    for (const auto& reservation : final_reservations) {
        const int loc = reservation.first;
        const int agent = reservation.second;
        if (exec_->is_valid_location(loc))
            exec_->tpg[loc].push_back(agent);
    }

    if ((int)exec_->previous_locations.size() != n)
        exec_->previous_locations.assign(n, -1);
    for (int i = 0; i < n; i++)
        exec_->previous_locations[i] = env_->system_states[i].location;

    exec_->official_process_initialized_ = true;
    return exec_->predicted_states;
}

void IncrementalTpgExecutor::next_command(
    int /*exec_time_limit*/,
    std::vector<ExecutionCommand>& agent_command)
{
    if (exec_ == nullptr || env_ == nullptr) return;
    for (int i = 0; i < (int)env_->system_states.size(); i++)
        exec_->mcp_default_executor(i, agent_command);
}

} // namespace optsvr
