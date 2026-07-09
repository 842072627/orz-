// optsvr/schedule/MazeOptScheduler/MazeOptScheduler.cpp
// Source: IDEA/MAZEOpt TaskScheduler::lorr26_dhfull_until_3000_then_mincostflow_top80_scheduler.
// Runtime-configured in MAIN2026 through OPT_MAZE_* knobs.
#include "MazeOptScheduler.h"

#include "RuntimeConfig.h"
#include "scheduler.h"
#include "const.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>

namespace orz_engine {
namespace {

struct MinCostFlowEdge {
    int to;
    int rev;
    int cap;
    double cost;
};

void add_min_cost_flow_edge(std::vector<std::vector<MinCostFlowEdge>>& graph,
                            int from,
                            int to,
                            int cap,
                            double cost)
{
    MinCostFlowEdge forward{to, static_cast<int>(graph[to].size()), cap, cost};
    MinCostFlowEdge backward{from, static_cast<int>(graph[from].size()), 0, -cost};
    graph[from].push_back(forward);
    graph[to].push_back(backward);
}

std::vector<int> solve_min_cost_assignment(const std::vector<std::vector<double>>& cost_matrix,
                                           int required_flow)
{
    const int agent_count = static_cast<int>(cost_matrix.size());
    const int task_count = agent_count == 0 ? 0 : static_cast<int>(cost_matrix.front().size());
    std::vector<int> assignment(agent_count, -1);
    if (agent_count == 0 || task_count == 0 || required_flow <= 0) return assignment;

    required_flow = std::min(required_flow, std::min(agent_count, task_count));
    const int source = 0;
    const int agent_base = 1;
    const int task_base = agent_base + agent_count;
    const int sink = task_base + task_count;
    std::vector<std::vector<MinCostFlowEdge>> graph(sink + 1);

    for (int i = 0; i < agent_count; ++i)
        add_min_cost_flow_edge(graph, source, agent_base + i, 1, 0.0);
    for (int j = 0; j < task_count; ++j)
        add_min_cost_flow_edge(graph, task_base + j, sink, 1, 0.0);
    for (int i = 0; i < agent_count; ++i) {
        for (int j = 0; j < task_count; ++j) {
            if (cost_matrix[i][j] < 5e8)
                add_min_cost_flow_edge(graph, agent_base + i, task_base + j, 1, cost_matrix[i][j]);
        }
    }

    constexpr double kInfCost = 1e100;
    int sent_flow = 0;
    while (sent_flow < required_flow) {
        std::vector<double> dist(graph.size(), kInfCost);
        std::vector<int> prev_node(graph.size(), -1);
        std::vector<int> prev_edge(graph.size(), -1);
        std::vector<bool> in_queue(graph.size(), false);
        std::queue<int> q;
        dist[source] = 0.0;
        q.push(source);
        in_queue[source] = true;

        while (!q.empty()) {
            const int node = q.front();
            q.pop();
            in_queue[node] = false;
            for (int e = 0; e < static_cast<int>(graph[node].size()); ++e) {
                const MinCostFlowEdge& edge = graph[node][e];
                if (edge.cap <= 0) continue;
                if (dist[node] + edge.cost < dist[edge.to] - 1e-9) {
                    dist[edge.to] = dist[node] + edge.cost;
                    prev_node[edge.to] = node;
                    prev_edge[edge.to] = e;
                    if (!in_queue[edge.to]) {
                        q.push(edge.to);
                        in_queue[edge.to] = true;
                    }
                }
            }
        }

        if (prev_node[sink] == -1) break;

        int add_flow = required_flow - sent_flow;
        for (int node = sink; node != source; node = prev_node[node])
            add_flow = std::min(add_flow, graph[prev_node[node]][prev_edge[node]].cap);
        for (int node = sink; node != source; node = prev_node[node]) {
            MinCostFlowEdge& edge = graph[prev_node[node]][prev_edge[node]];
            edge.cap -= add_flow;
            graph[node][edge.rev].cap += add_flow;
        }
        sent_flow += add_flow;
    }

    for (int i = 0; i < agent_count; ++i) {
        const int agent_node = agent_base + i;
        for (const MinCostFlowEdge& edge : graph[agent_node]) {
            if (edge.to >= task_base && edge.to < task_base + task_count && edge.cap == 0) {
                assignment[i] = edge.to - task_base;
                break;
            }
        }
    }
    return assignment;
}

double solve_hungarian(std::vector<std::vector<double>> cost_matrix,
                       std::vector<int>& assignment)
{
    int n = static_cast<int>(cost_matrix.size());
    if (n == 0) {
        assignment.clear();
        return 0.0;
    }
    int m = static_cast<int>(cost_matrix.front().size());
    if (m == 0) {
        assignment.assign(n, -1);
        return 0.0;
    }

    bool transpose = n > m;
    const int original_rows = n;
    const int original_cols = m;
    if (transpose) {
        std::vector<std::vector<double>> transposed(m, std::vector<double>(n, 0.0));
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                transposed[j][i] = cost_matrix[i][j];
        cost_matrix.swap(transposed);
        std::swap(n, m);
    }

    std::vector<double> u(n + 1, 0.0), v(m + 1, 0.0);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m + 1, std::numeric_limits<double>::max());
        std::vector<char> used(m + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            double delta = std::numeric_limits<double>::max();
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                const double cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> local_assignment(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] != 0) local_assignment[p[j] - 1] = j - 1;
    }

    if (!transpose) {
        assignment = std::move(local_assignment);
    } else {
        assignment.assign(original_rows, -1);
        for (int i = 0; i < n; ++i) {
            const int original_row = local_assignment[i];
            const int original_col = i;
            if (original_row >= 0 && original_row < original_rows &&
                original_col >= 0 && original_col < original_cols)
                assignment[original_row] = original_col;
        }
    }
    return -v[0];
}

} // namespace

void MazeOptScheduler::initialize(SharedEnvironment* env, int preprocess_time_limit_ms)
{
    env_ = env;
    const int limit = preprocess_time_limit_ms -
                      DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE - 1000;
    DefaultPlanner::schedule_initialize(std::max(1, limit), env_);

    agent_task_.assign(env_->num_of_agents, AgentTask());
    task_agent_.assign(110000, -1);
    free_agents_.clear();
    free_tasks_.clear();
    num_task_finished_ = 0;
    total_min_span_ = 0;
    total_real_duration_ = 0;
    total_jam_ = 0.0;

    const auto& c = cfg();
    std::cerr << "[MazeOptScheduler] initialized: switch_tick=" << c.opt_maze_switch_tick
              << " ratio=" << c.opt_maze_assign_ratio
              << " snatch_dist=" << c.opt_maze_snatch_min_pickup_distance
              << " cell=" << c.opt_maze_cell_size
              << " first_w=" << c.opt_maze_dist_first_w
              << " late_first_w=" << c.opt_maze_late_dist_first_w
              << " overall=" << c.opt_maze_dist_overall
              << " dens_start=" << c.opt_maze_density_start_w
              << " dens_end=" << c.opt_maze_density_end_w << "\n";
}

void MazeOptScheduler::schedule(int time_limit_ms,
                                std::vector<int>& proposed_schedule,
                                SharedEnvironment* env)
{
    env_ = env;
    if (!env_) return;
    lorr26_dhfull_until_switch_then_mincostflow_scheduler(
        time_limit_ms, proposed_schedule);
}

void MazeOptScheduler::ensure_task_agent_size(int task_id)
{
    if (task_id >= static_cast<int>(task_agent_.size()))
        task_agent_.resize(task_id + 1, -1);
}

int MazeOptScheduler::scheduler_distance(int source, int target) const
{
    return DefaultPlanner::get_h(env_, source, target);
}

int MazeOptScheduler::task_min_distance(int agent_id, int task_id) const
{
    auto task_it = env_->task_pool.find(task_id);
    if (task_it == env_->task_pool.end()) return 0;
    int dist = 0;
    int curr_loc = env_->curr_states.at(agent_id).location;
    for (int loc : task_it->second.locations) {
        dist += scheduler_distance(curr_loc, loc);
        curr_loc = loc;
    }
    return dist;
}

void MazeOptScheduler::lorr26_dhfull_scheduler(int time_limit_ms,
                                               std::vector<int>& proposed_schedule,
                                               double assignment_ratio,
                                               bool use_min_cost_flow)
{
    const TimePoint endtime =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit_ms);

    for (const auto& agent : env_->new_freeagents) {
        if (agent >= 0 && agent < static_cast<int>(agent_task_.size())) {
            agent_task_[agent].complete_moment = env_->curr_timestep;

            if (agent_task_[agent].task_id != -1) {
                num_task_finished_++;
                total_min_span_ += agent_task_[agent].min_task_dist;
                total_real_duration_ +=
                    agent_task_[agent].complete_moment - agent_task_[agent].assign_moment;
                total_jam_ += agent_task_[agent].jam_when_assign;
            }

            agent_task_[agent].task_id = -1;
        }
    }

    free_agents_.insert(env_->new_freeagents.begin(), env_->new_freeagents.end());
    free_tasks_.insert(env_->new_tasks.begin(), env_->new_tasks.end());

    std::fill(task_agent_.begin(), task_agent_.end(), -1);
    for (int agent = 0; agent < static_cast<int>(env_->curr_task_schedule.size()); ++agent) {
        const int task_id = env_->curr_task_schedule[agent];
        if (task_id != -1) {
            ensure_task_agent_size(task_id);
            task_agent_[task_id] = agent;
        }
    }

    std::unordered_set<int> busy_tasks;
    std::unordered_set<int> stealable_agents;
    std::unordered_set<int> stealable_tasks;
    for (int agent = 0; agent < static_cast<int>(env_->curr_task_schedule.size()); agent++) {
        const int task_id = env_->curr_task_schedule[agent];
        if (task_id == -1) continue;

        busy_tasks.insert(task_id);
        free_agents_.erase(agent);

        auto task_it = env_->task_pool.find(task_id);
        if (task_it != env_->task_pool.end() &&
            task_agent_[task_id] == agent &&
            task_it->second.idx_next_loc == 0 &&
            !task_it->second.locations.empty()) {
            const int next_loc = task_it->second.locations.front();
            const int agent_loc = env_->curr_states.at(agent).location;
            if (scheduler_distance(agent_loc, next_loc) >
                cfg().opt_maze_snatch_min_pickup_distance) {
                stealable_agents.insert(agent);
                stealable_tasks.insert(task_id);
            }
        }
    }

    for (int task_id : busy_tasks)
        free_tasks_.erase(task_id);

    for (auto it = free_tasks_.begin(); it != free_tasks_.end();) {
        auto task_it = env_->task_pool.find(*it);
        ensure_task_agent_size(*it);
        if (task_it == env_->task_pool.end() ||
            task_agent_[*it] != -1 ||
            task_it->second.locations.empty()) {
            it = free_tasks_.erase(it);
        } else {
            ++it;
        }
    }

    std::unordered_set<int> candidate_agent_set(free_agents_.begin(), free_agents_.end());
    candidate_agent_set.insert(stealable_agents.begin(), stealable_agents.end());
    std::unordered_set<int> candidate_task_set(free_tasks_.begin(), free_tasks_.end());
    candidate_task_set.insert(stealable_tasks.begin(), stealable_tasks.end());

    if (candidate_agent_set.empty() || candidate_task_set.empty()) {
        for (int agent : free_agents_)
            proposed_schedule[agent] = -1;
        return;
    }

    std::vector<int> agents(candidate_agent_set.begin(), candidate_agent_set.end());
    std::vector<int> tasks(candidate_task_set.begin(), candidate_task_set.end());
    std::sort(agents.begin(), agents.end());
    std::sort(tasks.begin(), tasks.end());

    if (agents.empty() || tasks.empty()) return;

    free_agents_.clear();
    free_agents_.insert(agents.begin(), agents.end());
    free_tasks_.clear();
    free_tasks_.insert(tasks.begin(), tasks.end());

    std::vector<std::vector<double>> cost_matrix(
        agents.size(), std::vector<double>(tasks.size(), 0.0));
    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        if (std::chrono::steady_clock::now() > endtime) break;
        const int agent_id = agents[i];
        const int agent_loc = env_->curr_states.at(agent_id).location;

        for (int j = 0; j < static_cast<int>(tasks.size()); ++j) {
            if (std::chrono::steady_clock::now() > endtime) break;
            const int task_id = tasks[j];
            auto task_it = env_->task_pool.find(task_id);
            if (task_it == env_->task_pool.end() || task_it->second.locations.empty()) {
                cost_matrix[i][j] = 1e9;
                continue;
            }

            const Task& task = task_it->second;
            const int next_idx = std::max(0, task.idx_next_loc);
            if (next_idx >= static_cast<int>(task.locations.size())) {
                cost_matrix[i][j] = 1e9;
                continue;
            }

            const int first_loc = task.locations[next_idx];
            const auto& c = cfg();
            const float first_w =
                (env_ != nullptr && env_->system_timestep >= c.opt_maze_switch_tick)
                    ? c.opt_maze_late_dist_first_w
                    : c.opt_maze_dist_first_w;
            double total_dist =
                scheduler_distance(agent_loc, first_loc) * first_w;
            int current_loc = first_loc;
            int path_dist = 0;
            for (int k = next_idx + 1; k < static_cast<int>(task.locations.size()); ++k) {
                const int loc = task.locations[k];
                path_dist += scheduler_distance(current_loc, loc);
                current_loc = loc;
            }
            total_dist += path_dist;

            const int end_loc = task.locations.back();
            const int density_cell_size = std::max(1, c.opt_maze_cell_size);
            const int start_grid_r = (first_loc / env_->cols) / density_cell_size;
            const int start_grid_c = (first_loc % env_->cols) / density_cell_size;
            const int end_grid_r = (end_loc / env_->cols) / density_cell_size;
            const int end_grid_c = (end_loc % env_->cols) / density_cell_size;
            int density_penalty = 0;
            int density_penalty_end = 0;

            for (const auto& other : env_->task_pool) {
                if (other.first == task_id || other.second.locations.empty()) continue;
                const int other_start = other.second.locations.front();
                const int other_end = other.second.locations.back();
                const int other_start_grid_r = (other_start / env_->cols) / density_cell_size;
                const int other_start_grid_c = (other_start % env_->cols) / density_cell_size;
                const int other_end_grid_r = (other_end / env_->cols) / density_cell_size;
                const int other_end_grid_c = (other_end % env_->cols) / density_cell_size;

                if (std::abs(other_start_grid_r - start_grid_r) <= 1 &&
                    std::abs(other_start_grid_c - start_grid_c) <= 1)
                    density_penalty++;
                if (std::abs(other_end_grid_r - end_grid_r) <= 1 &&
                    std::abs(other_end_grid_c - end_grid_c) <= 1)
                    density_penalty_end++;
            }

            cost_matrix[i][j] =
                c.opt_maze_dist_overall * total_dist +
                c.opt_maze_density_start_w * density_penalty +
                c.opt_maze_density_end_w * density_penalty_end;
        }
    }

    const int real_assignment_limit = std::max(0, std::min(
        static_cast<int>(tasks.size()),
        static_cast<int>(static_cast<double>(agents.size()) * assignment_ratio)));
    if (real_assignment_limit == 0) {
        for (int agent_id : free_agents_)
            proposed_schedule[agent_id] = -1;
        return;
    }

    std::vector<int> assignment;
    if (use_min_cost_flow) {
        assignment = solve_min_cost_assignment(cost_matrix, real_assignment_limit);
    } else {
        const int dummy_task_count =
            std::max(0, static_cast<int>(agents.size()) - real_assignment_limit);
        if (dummy_task_count > 0) {
            for (auto& row : cost_matrix)
                row.insert(row.end(), dummy_task_count, 0.0);
        }
        solve_hungarian(cost_matrix, assignment);
    }

    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        const int agent_id = agents[i];
        const int task_idx = i < static_cast<int>(assignment.size()) ? assignment[i] : -1;
        if (task_idx >= 0 && task_idx < static_cast<int>(tasks.size())) {
            const int task_id = tasks[task_idx];
            if (free_agents_.count(agent_id) && free_tasks_.count(task_id)) {
                const int old_task_id =
                    agent_id >= 0 && agent_id < static_cast<int>(env_->curr_task_schedule.size())
                        ? env_->curr_task_schedule[agent_id]
                        : -1;
                if (old_task_id != -1) {
                    ensure_task_agent_size(old_task_id);
                    if (task_agent_[old_task_id] == agent_id)
                        task_agent_[old_task_id] = -1;
                }

                ensure_task_agent_size(task_id);
                task_agent_[task_id] = agent_id;

                proposed_schedule[agent_id] = task_id;
                free_agents_.erase(agent_id);
                free_tasks_.erase(task_id);

                if (agent_id >= 0 && agent_id < static_cast<int>(agent_task_.size())) {
                    agent_task_[agent_id].task_id = task_id;
                    agent_task_[agent_id].assign_moment = env_->curr_timestep;
                    agent_task_[agent_id].min_task_dist =
                        task_min_distance(agent_id, task_id);
                    agent_task_[agent_id].jam_when_assign = 0;
                }
            }
        } else if (free_agents_.count(agent_id)) {
            proposed_schedule[agent_id] = -1;
        }
    }

    for (int agent_id : free_agents_)
        proposed_schedule[agent_id] = -1;

}

void MazeOptScheduler::lorr26_dhfull_mincostflow_scheduler(
    int time_limit_ms,
    std::vector<int>& proposed_schedule)
{
    lorr26_dhfull_scheduler(time_limit_ms, proposed_schedule,
                            cfg().opt_maze_assign_ratio, true);
}

void MazeOptScheduler::lorr26_dhfull_until_switch_then_mincostflow_scheduler(
    int time_limit_ms,
    std::vector<int>& proposed_schedule)
{
    if (env_ != nullptr && env_->system_timestep >= cfg().opt_maze_switch_tick) {
        lorr26_dhfull_mincostflow_scheduler(time_limit_ms, proposed_schedule);
        return;
    }

    lorr26_dhfull_scheduler(time_limit_ms, proposed_schedule);
}

} // namespace orz_engine
