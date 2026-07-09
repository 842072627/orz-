// optsvr/schedule/DHSchedule/DHScheduler.cpp
// 来源：IDEA/Schedule-DH/TaskScheduler.cpp（保留原始结构，用类封装）
// 包含：initialize / plan(sortation/room_small/maze路由) / schedule_plan(maze大图) / 工具函数
#include "DHScheduler.h"
#include "HeuristicMatrix.h"
#include "heuristics.h"
#include "const.h"
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_set>

namespace optsvr {

// ════════════════════════════════════════════════════════════════════
// 距离查询（替代原始 PrecomputedHeuristicTable）
// ════════════════════════════════════════════════════════════════════
int DHScheduler::scheduler_distance(int source, int target) const {
    auto& hm = heuristic_matrix();
    if (hm.is_ready()) {
        uint16_t d = hm.get(source, target);
        if (d != DIST_UNREACHABLE) return static_cast<int>(d);
    }
    return DefaultPlanner::get_h(env_, source, target);
}

int DHScheduler::task_path_cost(int agent_loc, const Task& task) const {
    if (task.locations.empty()) return 0;
    int total_dist = scheduler_distance(agent_loc, task.locations[0]);
    int current_loc = task.locations[0];
    for (size_t i = 1; i < task.locations.size(); ++i) {
        total_dist += scheduler_distance(current_loc, task.locations[i]);
        current_loc = task.locations[i];
    }
    return total_dist;
}

// ════════════════════════════════════════════════════════════════════
// 辅助函数（原始 anonymous namespace 中的内容）
// ════════════════════════════════════════════════════════════════════
int DHScheduler::computeDynamicRadius(int num_free_agents, int num_free_tasks) {
    if (num_free_agents == 0) return BASE_RADIUS;
    double density_ratio = static_cast<double>(num_free_tasks) / static_cast<double>(num_free_agents);
    int dynamic_radius;
    if (density_ratio < DENSITY_LOW_THRESHOLD) {
        double scale = 1.0 + log(1.0 / std::max(density_ratio, 0.1));
        dynamic_radius = static_cast<int>(BASE_RADIUS * scale);
    } else if (density_ratio > DENSITY_HIGH_THRESHOLD) {
        double scale = 1.0 / (1.0 + log(1.0 + density_ratio));
        dynamic_radius = static_cast<int>(BASE_RADIUS * scale);
    } else {
        dynamic_radius = BASE_RADIUS;
    }
    return std::max(MIN_RADIUS, std::min(MAX_RADIUS, dynamic_radius));
}

int DHScheduler::computeMakespanHelper(int agent_id, int task_id) {
    int dist = 0;
    int c_loc = env_->curr_states.at(agent_id).location;
    const auto& task = env_->task_pool[task_id];
    for (int loc : task.locations) {
        dist += DefaultPlanner::get_h(env_, c_loc, loc);
        c_loc = loc;
    }
    return dist;
}

int DHScheduler::hungarianMinMatch(const std::vector<std::vector<int>>& cost, std::vector<int>& assignment) {
    int n = cost.size();
    int m = cost[0].size();
    assignment.assign(n, -1);
    if (n == 0 || m == 0) return 0;
    if (n > m) return 0;

    std::vector<int> u(n+1), v(m+1), p(m+1), way(m+1);
    for (int i = 1; i <= n; ++i) {
        p[0] = i; int j0 = 0;
        std::vector<int> minv(m+1, std::numeric_limits<int>::max());
        std::vector<bool> used(m+1, false);
        do {
            used[j0] = true; int i0 = p[j0], delta = std::numeric_limits<int>::max(), j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    int cur = cost[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
    }
    for (int j = 1; j <= m; ++j)
        if (p[j] > 0 && p[j] <= n) assignment[p[j]-1] = j-1;
    return -v[0];
}

double DHScheduler::hungarianSolve(std::vector<std::vector<double>>& cost_matrix, std::vector<int>& assignment) {
    int n = cost_matrix.size();
    if (n == 0) return 0.0;
    int m = cost_matrix[0].size();
    if (m == 0) return 0.0;

    bool transpose = n > m;
    if (transpose) {
        std::swap(n, m);
        std::vector<std::vector<double>> temp(n, std::vector<double>(m));
        for (int i = 0; i < n; ++i) for (int j = 0; j < m; ++j) temp[i][j] = cost_matrix[j][i];
        cost_matrix.swap(temp);
    }

    std::vector<double> u(n+1, 0), v(m+1, 0);
    std::vector<int> p(m+1, 0), way(m+1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i; int j0 = 0;
        std::vector<double> minv(m+1, std::numeric_limits<double>::max());
        std::vector<bool> used(m+1, false);
        do {
            used[j0] = true; int i0 = p[j0], j1 = 0;
            double delta = std::numeric_limits<double>::max();
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    double cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0 != 0);
    }

    assignment.assign(n, -1);
    for (int j = 1; j <= m; ++j) if (p[j] != 0) assignment[p[j]-1] = j-1;
    if (transpose) {
        std::vector<int> tmp(m, -1);
        for (int i = 0; i < n; ++i) if (assignment[i] != -1) tmp[assignment[i]] = i;
        assignment.swap(tmp);
    }
    return -v[0];
}

// ── reclaim 相关 ──
int DHScheduler::optimistic_ticks_to_first_loc(int agent_id, const Task& task) {
    if (task.locations.empty() || agent_id < 0 || agent_id >= (int)env_->system_states.size())
        return std::numeric_limits<int>::max();
    const State& state = env_->system_states.at(agent_id);
    int optimistic_loc = state.location; int remaining_ticks = 0;
    if (state.counter.count > 0 && state.current_action != Action::NA) {
        remaining_ticks = std::max(0, state.counter.maxCount - state.counter.count);
        if (state.current_action == Action::FW) {
            const int moves[4] = {1, env_->cols, -1, -env_->cols};
            optimistic_loc += moves[state.orientation];
        }
    }
    return remaining_ticks + scheduler_distance(optimistic_loc, task.locations[0]) * env_->max_counter;
}

bool DHScheduler::can_reclaim_task(int agent_id, const Task& task) {
    if (task.idx_next_loc != 0 || task.locations.empty() || task.agent_assigned != agent_id) return false;
    if (agent_id < 0 || agent_id >= (int)env_->curr_task_schedule.size() ||
        env_->curr_task_schedule[agent_id] != task.task_id) return false;
    const int action_time = std::max(1, env_->action_time);
    int horizon_ticks = (env_->min_planner_communication_time + action_time - 1) / action_time + 1;
    return optimistic_ticks_to_first_loc(agent_id, task) > horizon_ticks;
}

void DHScheduler::rebuild_scheduler_state(std::vector<int>& proposed_schedule) {
    free_agents_.clear(); free_tasks_.clear(); task_assignments_.clear();
    for (int agent_id = 0; agent_id < (int)env_->curr_task_schedule.size(); ++agent_id) {
        if (env_->curr_task_schedule[agent_id] == -1) { free_agents_.insert(agent_id); proposed_schedule[agent_id] = -1; }
    }
    for (const auto& [task_id, task] : env_->task_pool) {
        if (task.agent_assigned == -1) free_tasks_.insert(task_id);
        else task_assignments_[task_id] = task.agent_assigned;
    }
}

void DHScheduler::reclaim_safe_tasks(std::vector<int>& proposed_schedule) {
    for (auto it = task_assignments_.begin(); it != task_assignments_.end(); ) {
        auto task_it = env_->task_pool.find(it->first);
        if (task_it == env_->task_pool.end()) { it = task_assignments_.erase(it); continue; }
        if (can_reclaim_task(it->second, task_it->second)) {
            free_tasks_.insert(it->first); free_agents_.insert(it->second);
            proposed_schedule[it->second] = -1; it = task_assignments_.erase(it);
        } else { ++it; }
    }
}

// ════════════════════════════════════════════════════════════════════
// schedule_plan — 对应原始 TaskScheduler::schedule_plan（maze/大图逻辑）
// ════════════════════════════════════════════════════════════════════
void DHScheduler::schedule_plan(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    rebuild_scheduler_state(proposed_schedule);

    // 分支1: num_of_agents < 500
    if (env_->num_of_agents < 500) {
        const int CELL_SIZE_R = 2;
        if (free_agents_.size() < 300) {
            reclaim_safe_tasks(proposed_schedule);
            std::vector<int> agents(free_agents_.begin(), free_agents_.end());
            std::vector<int> tasks(free_tasks_.begin(), free_tasks_.end());
            int num_agents = agents.size(), num_tasks = tasks.size();
            if (num_agents == 0 || num_tasks == 0) { for (int aid : free_agents_) proposed_schedule[aid] = -1; return; }

            std::vector<std::vector<double>> cost_matrix(num_agents, std::vector<double>(num_tasks, 0.0));
            for (int i = 0; i < num_agents; ++i) {
                if (std::chrono::steady_clock::now() > endtime) break;
                int agent_id = agents[i];
                for (int j = 0; j < num_tasks; ++j) {
                    if (std::chrono::steady_clock::now() > endtime) break;
                    int task_id = tasks[j];
                    if (!env_->task_pool.count(task_id)) continue;
                    const Task& task = env_->task_pool.at(task_id);
                    int agent_loc = env_->curr_states.at(agent_id).location;

                    double total_dist = scheduler_distance(agent_loc, task.locations[0]) * 6;
                    int current_loc = agent_loc;
                    for (const auto& step : task.locations) { total_dist += scheduler_distance(current_loc, step); current_loc = step; }

                    int start_loc = task.locations[0], end_loc = task.locations.back();
                    int grid_x = (start_loc / env_->cols) / CELL_SIZE_R, grid_y = (start_loc % env_->cols) / CELL_SIZE_R;
                    int egrid_x = (end_loc / env_->cols) / CELL_SIZE_R, egrid_y = (end_loc % env_->cols) / CELL_SIZE_R;

                    int density_penalty = 0, density_penaltyend = 0;
                    for (const auto& other : env_->task_pool) {
                        if (other.first == task_id) continue;
                        int os = other.second.locations[0];
                        int ox = (os / env_->cols) / CELL_SIZE_R, oy = (os % env_->cols) / CELL_SIZE_R;
                        if (abs(ox - grid_x) <= 1 && abs(oy - grid_y) <= 1) density_penalty++;
                        if (abs(ox - egrid_x) <= 1 && abs(oy - egrid_y) <= 1) density_penaltyend++;
                    }
                    double score = -(0.9 * total_dist - 0.1 * density_penalty + 0.2 * density_penaltyend);
                    cost_matrix[i][j] = -score;
                }
            }
            std::vector<int> assignment;
            hungarianSolve(cost_matrix, assignment);
            for (int i = 0; i < num_agents; ++i) {
                if (assignment[i] != -1 && assignment[i] < num_tasks) {
                    int agent_id = agents[i], task_id = tasks[assignment[i]];
                    if (free_agents_.count(agent_id) && free_tasks_.count(task_id)) {
                        task_assignments_[task_id] = agent_id;
                        proposed_schedule[agent_id] = task_id;
                        free_agents_.erase(agent_id); free_tasks_.erase(task_id);
                    }
                }
            }
            for (int aid : free_agents_) proposed_schedule[aid] = -1;
        } else {
            using TaskAssignment = std::tuple<int, int, int>;
            std::priority_queue<TaskAssignment, std::vector<TaskAssignment>, std::greater<>> assignment_queue;
            for (int agent_id : free_agents_) {
                for (int task_id : free_tasks_) {
                    if (std::chrono::steady_clock::now() > endtime) break;
                    int dist = task_path_cost(env_->curr_states.at(agent_id).location, env_->task_pool[task_id])
                             + 5 * scheduler_distance(env_->curr_states.at(agent_id).location, env_->task_pool[task_id].locations[0]);
                    assignment_queue.emplace(dist, agent_id, task_id);
                }
            }
            while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
                auto [makespan, agent_id, task_id] = assignment_queue.top(); assignment_queue.pop();
                if (free_agents_.count(agent_id) && free_tasks_.count(task_id)) {
                    task_assignments_[task_id] = agent_id;
                    proposed_schedule[agent_id] = task_id;
                    free_agents_.erase(agent_id); free_tasks_.erase(task_id);
                }
            }
            for (int aid : free_agents_) proposed_schedule[aid] = -1;
        }
        return;
    }

    // 分支2: num_of_agents >= 500
    if (free_agents_.size() < 110) {
        const double congestion_penalty = 0.1;
        const size_t num_free_agents = free_agents_.size(), num_free_tasks = free_tasks_.size();
        const double ratio = num_free_tasks > 0 ? (double)num_free_agents / num_free_tasks : 1.0;
        const double midpoint = 1.0, steepness = 5.0;
        const double alpha = 1.0 / (1.0 + exp(-steepness * (ratio - midpoint)));
        const double weight_dist = 0.5 * (1.0 - alpha), weight_cluster = 0.2 * alpha;

        using TaskAssignment = std::tuple<double, int, int>;
        std::priority_queue<TaskAssignment> assignment_queue;
        int count = 0;
        for (int agent_id : free_agents_) {
            if (std::chrono::steady_clock::now() > endtime) break;
            for (int task_id : free_tasks_) {
                if (count % 20 == 0 && std::chrono::steady_clock::now() > endtime) break;
                if (!env_->task_pool.count(task_id)) continue;
                const Task& task = env_->task_pool.at(task_id);
                int agent_loc = env_->curr_states.at(agent_id).location;

                double total_dist = scheduler_distance(agent_loc, task.locations[0]) * 10;
                int cur_loc = task.locations[0];
                for (size_t i = 1; i < task.locations.size(); i++) { total_dist += scheduler_distance(cur_loc, task.locations[i]); cur_loc = task.locations[i]; }
                total_dist += congestion_penalty * location_congestion_[agent_loc];
                for (const auto& loc : task.locations) total_dist += congestion_penalty * location_congestion_[loc];

                double cluster_factor = 0; int cluster_count = 0;
                for (const auto& other : env_->task_pool) {
                    if (other.first == task_id) continue;
                    double min_dist = std::numeric_limits<double>::max();
                    for (const auto& loc1 : task.locations) { double d = scheduler_distance(loc1, other.second.locations[0]); if (d < min_dist) min_dist = d; }
                    cluster_factor += min_dist; cluster_count++;
                }
                cluster_factor = cluster_count > 0 ? cluster_factor / cluster_count : 0;
                assignment_queue.emplace(-(weight_dist * total_dist - weight_cluster * cluster_factor), agent_id, task_id);
                count++;
            }
        }
        while (!assignment_queue.empty()) {
            auto [score, agent_id, task_id] = assignment_queue.top(); assignment_queue.pop();
            if (free_agents_.count(agent_id) && free_tasks_.count(task_id)) {
                task_assignments_[task_id] = agent_id;
                proposed_schedule[agent_id] = task_id;
                free_agents_.erase(agent_id); free_tasks_.erase(task_id);
                location_congestion_[env_->curr_states[agent_id].location]++;
                for (const auto& loc : env_->task_pool[task_id].locations) location_congestion_[loc]++;
            }
        }
        for (int aid : free_agents_) proposed_schedule[aid] = -1;

    } else if (free_agents_.size() < 1000) {
        using TaskAssignment = std::tuple<int, int, int>;
        std::priority_queue<TaskAssignment, std::vector<TaskAssignment>, std::greater<>> assignment_queue;
        for (int agent_id : free_agents_) {
            for (int task_id : free_tasks_) {
                if (std::chrono::steady_clock::now() > endtime) break;
                int dist = task_path_cost(env_->curr_states.at(agent_id).location, env_->task_pool[task_id])
                         + 7 * scheduler_distance(env_->curr_states.at(agent_id).location, env_->task_pool[task_id].locations[0]);
                assignment_queue.emplace(dist, agent_id, task_id);
            }
        }
        while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
            auto [makespan, agent_id, task_id] = assignment_queue.top(); assignment_queue.pop();
            if (free_agents_.count(agent_id) && free_tasks_.count(task_id)) {
                task_assignments_[task_id] = agent_id;
                proposed_schedule[agent_id] = task_id;
                free_agents_.erase(agent_id); free_tasks_.erase(task_id);
            }
        }
        for (int aid : free_agents_) proposed_schedule[aid] = -1;

    } else {
        auto it = free_agents_.begin();
        while (it != free_agents_.end()) {
            if (std::chrono::steady_clock::now() > endtime) break;
            int i = *it;
            int min_task_i = -1, min_task_makespan = INT_MAX, count = 0;
            for (int t_id : free_tasks_) {
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                int dist = task_path_cost(env_->curr_states.at(i).location, env_->task_pool[t_id])
                         + 7 * scheduler_distance(env_->curr_states.at(i).location, env_->task_pool[t_id].locations[0]);
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
            if (min_task_i != -1) { task_assignments_[min_task_i] = i; proposed_schedule[i] = min_task_i; it = free_agents_.erase(it); free_tasks_.erase(min_task_i); }
            else { proposed_schedule[i] = -1; ++it; }
        }
    }
}

// ════════════════════════════════════════════════════════════════════
// initialize — 对应原始 TaskScheduler::initialize
// ════════════════════════════════════════════════════════════════════
void DHScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    env_ = env;
    free_agents_.clear(); free_tasks_.clear();
    task_assignments_.clear(); location_congestion_.clear();
    clusters_.clear(); task_to_cluster_.clear();
    last_clustered_free_tasks_count_ = 0; rng_.seed(42);
    // 注：原始代码在此处调用 PrecomputedHeuristicTable::initialize(env)
    //     现已由框架层 HeuristicMatrix 在 OptScheduler 或 Entry 中统一预计算
    if (env_->map_name[0] == 'o' || env_->map_name[0] == 'w') {
        initQuadTree();
    }
}

// ════════════════════════════════════════════════════════════════════
// schedule — 对应原始 TaskScheduler::plan（路由入口）
// ════════════════════════════════════════════════════════════════════
void DHScheduler::schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    env_ = env;

    if (env_->map_name[0] == 's') {
        // ── Sortation: 动态半径聚类 + 局部匈牙利 ──
        int limit = time_limit_ms/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

        free_agents_.insert(env_->new_freeagents.begin(), env_->new_freeagents.end());
        free_tasks_.insert(env_->new_tasks.begin(), env_->new_tasks.end());

        int dynamic_radius = computeDynamicRadius(free_agents_.size(), free_tasks_.size());
        int extended_radius = dynamic_radius * 2;

        while (!free_agents_.empty()) {
            if (std::chrono::steady_clock::now() > endtime) break;
            int seed_agent = *free_agents_.begin();
            int seed_loc = env_->curr_states[seed_agent].location;

            std::vector<int> cluster_agents = {seed_agent};
            std::vector<int> cluster_tasks;

            std::vector<std::pair<int,int>> task_distances;
            task_distances.reserve(free_tasks_.size());
            for (int t_id : free_tasks_) {
                int d = DefaultPlanner::get_h(env_, seed_loc, env_->task_pool[t_id].locations.front());
                task_distances.push_back({d, t_id});
            }
            std::sort(task_distances.begin(), task_distances.end());

            std::unordered_set<int> candidate_task_set;
            int max_candidate_tasks = std::min((int)task_distances.size(), MAX_CLUSTER_TASKS + 5);
            for (int i = 0; i < max_candidate_tasks; ++i) candidate_task_set.insert(task_distances[i].second);

            for (int other_agent : free_agents_) {
                if ((int)cluster_agents.size() >= MAX_CLUSTER_AGENTS) break;
                if (other_agent == seed_agent) continue;
                int other_loc = env_->curr_states[other_agent].location;
                for (int t_id : candidate_task_set) {
                    int d = DefaultPlanner::get_h(env_, other_loc, env_->task_pool[t_id].locations.front());
                    if (d < dynamic_radius) { cluster_agents.push_back(other_agent); break; }
                }
            }

            for (int t_id : free_tasks_) {
                if ((int)cluster_tasks.size() >= MAX_CLUSTER_TASKS) break;
                int min_dist_to_cluster = std::numeric_limits<int>::max();
                for (int a_id : cluster_agents) {
                    int d = DefaultPlanner::get_h(env_, env_->curr_states[a_id].location, env_->task_pool[t_id].locations.front());
                    min_dist_to_cluster = std::min(min_dist_to_cluster, d);
                    if (min_dist_to_cluster < dynamic_radius) break;
                }
                if (min_dist_to_cluster < extended_radius) cluster_tasks.push_back(t_id);
            }

            if (cluster_agents.size() <= 1 || cluster_tasks.size() <= 1) {
                int best_task = -1, min_makespan = std::numeric_limits<int>::max(), count = 0;
                for (int t_id : free_tasks_) {
                    if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                    int makespan = computeMakespanHelper(seed_agent, t_id);
                    if (makespan < min_makespan) { min_makespan = makespan; best_task = t_id; }
                    count++;
                }
                if (best_task != -1) { proposed_schedule[seed_agent] = best_task; free_tasks_.erase(best_task); }
                else proposed_schedule[seed_agent] = -1;
                free_agents_.erase(seed_agent);
            } else {
                int n_a = (int)cluster_agents.size(), n_t = (int)cluster_tasks.size();
                std::vector<std::vector<int>> cost_matrix(n_a, std::vector<int>(n_t));
                for (int i = 0; i < n_a; ++i) for (int j = 0; j < n_t; ++j) cost_matrix[i][j] = computeMakespanHelper(cluster_agents[i], cluster_tasks[j]);
                std::vector<int> assignment;
                hungarianMinMatch(cost_matrix, assignment);
                for (int i = 0; i < n_a; ++i) {
                    int a = cluster_agents[i];
                    if (i < (int)assignment.size() && assignment[i] >= 0 && assignment[i] < n_t) { proposed_schedule[a] = cluster_tasks[assignment[i]]; free_tasks_.erase(cluster_tasks[assignment[i]]); }
                    else proposed_schedule[a] = -1;
                    free_agents_.erase(a);
                }
            }
        }
        for (int a : free_agents_) proposed_schedule[a] = -1;
        free_agents_.clear();
        return;
    }

    // r 图：>300 走 K-means(planLargeR)
    if (env_->num_of_agents > 300 && env_->map_name[0] == 'r') {
        planLargeR(time_limit_ms, proposed_schedule);
        return;
    }

    // o/w 图
    if (env_->map_name[0] == 'o' || env_->map_name[0] == 'w') {
        planOW(time_limit_ms, proposed_schedule);
        return;
    }

    // r 图 < 300: Top-K 竞标 + yield
    if (env_->num_of_agents < 300 && env_->map_name[0] == 'r') {
        int limit = time_limit_ms/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

        free_agents_.insert(env_->new_freeagents.begin(), env_->new_freeagents.end());
        free_tasks_.insert(env_->new_tasks.begin(), env_->new_tasks.end());
        for (auto it = free_agents_.begin(); it != free_agents_.end(); ) {
            int a = *it;
            if (a < 0 || a >= (int)env_->curr_task_schedule.size() || env_->curr_task_schedule[a] != -1) it = free_agents_.erase(it);
            else { proposed_schedule[a] = -1; ++it; }
        }
        for (auto it = free_tasks_.begin(); it != free_tasks_.end(); ) {
            auto ti = env_->task_pool.find(*it);
            if (ti == env_->task_pool.end() || ti->second.agent_assigned != -1 || ti->second.locations.empty()) it = free_tasks_.erase(it);
            else ++it;
        }
        if (free_agents_.empty() || free_tasks_.empty()) return;

        struct AgentBid {
            int agent_id; std::vector<std::pair<int,int>> top_k; int curr_idx = 0;
            int get_best_available(const std::unordered_set<int>& occ) { while (curr_idx < (int)top_k.size()) { if (occ.find(top_k[curr_idx].first) == occ.end()) return top_k[curr_idx].first; curr_idx++; } return -1; }
            int get_best_cost(const std::unordered_set<int>& occ) { int t = get_best_available(occ); if (t == -1) return INT_MAX; return top_k[curr_idx].second; }
            int yield_cost(const std::unordered_set<int>& occ) { int si = curr_idx; int old_cost = get_best_cost(occ); curr_idx++; int new_cost = get_best_cost(occ); curr_idx = si; if (new_cost == INT_MAX) return INT_MAX; return new_cost - old_cost; }
        };

        std::vector<AgentBid> bids; bids.reserve(free_agents_.size());
        for (int agent_id : free_agents_) {
            if (std::chrono::steady_clock::now() > endtime) break;
            AgentBid bid; bid.agent_id = agent_id;
            int start_loc = env_->curr_states.at(agent_id).location;
            for (int task_id : free_tasks_) {
                if (std::chrono::steady_clock::now() > endtime) break;
                auto task_it = env_->task_pool.find(task_id);
                if (task_it == env_->task_pool.end() || task_it->second.locations.empty()) continue;
                int dist = 0, curr_loc = start_loc;
                for (int loc : task_it->second.locations) { dist += DefaultPlanner::get_h(env_, curr_loc, loc); curr_loc = loc; }
                if ((int)bid.top_k.size() < K_BEST_TASKS) { auto it2 = bid.top_k.begin(); while (it2 != bid.top_k.end() && it2->second < dist) ++it2; bid.top_k.insert(it2, {task_id, dist}); }
                else if (dist < bid.top_k.back().second) { auto it2 = bid.top_k.begin(); while (it2 != bid.top_k.end() && it2->second < dist) ++it2; bid.top_k.insert(it2, {task_id, dist}); bid.top_k.pop_back(); }
            }
            if (!bid.top_k.empty()) bids.push_back(bid);
        }
        if (bids.empty()) return;

        std::vector<int> agent_assignment(bids.size(), -1);
        std::unordered_map<int, std::vector<int>> task_claimants;
        std::unordered_set<int> occupied;
        for (int i = 0; i < (int)bids.size(); ++i) { int t = bids[i].get_best_available(occupied); if (t != -1) { agent_assignment[i] = t; task_claimants[t].push_back(i); occupied.insert(t); } }

        const int MAX_ITER = (int)bids.size() * 3;
        for (int iteration = 0; iteration < MAX_ITER && std::chrono::steady_clock::now() < endtime; ++iteration) {
            std::vector<int> conflicted;
            for (auto& entry : task_claimants) if (entry.second.size() > 1) conflicted.push_back(entry.first);
            if (conflicted.empty()) break;
            for (int task_id : conflicted) {
                auto& claimants = task_claimants[task_id]; if (claimants.size() <= 1) continue;
                int min_yield = INT_MAX, yield_idx = -1;
                for (int idx : claimants) { std::unordered_set<int> cur_occ; for (int at : agent_assignment) if (at != -1) cur_occ.insert(at); int yc = bids[idx].yield_cost(cur_occ); if (yc < min_yield) { min_yield = yc; yield_idx = idx; } }
                if (yield_idx == -1) continue;
                auto& old_cl = task_claimants[task_id];
                for (auto ci = old_cl.begin(); ci != old_cl.end(); ++ci) { if (*ci == yield_idx) { old_cl.erase(ci); break; } }
                if (old_cl.empty()) task_claimants.erase(task_id);
                std::unordered_set<int> rem_occ; for (auto& e : task_claimants) for (int idx : e.second) rem_occ.insert(e.first);
                int new_task = bids[yield_idx].get_best_available(rem_occ);
                if (new_task != -1) { agent_assignment[yield_idx] = new_task; task_claimants[new_task].push_back(yield_idx); }
                else agent_assignment[yield_idx] = -1;
            }
        }

        for (int i = 0; i < (int)bids.size(); ++i) {
            int t = agent_assignment[i];
            if (t != -1 && env_->task_pool.find(t) != env_->task_pool.end()) { proposed_schedule[bids[i].agent_id] = t; free_agents_.erase(bids[i].agent_id); free_tasks_.erase(t); }
            else { proposed_schedule[bids[i].agent_id] = -1; free_agents_.erase(bids[i].agent_id); }
        }
        return;
    }

    // 其他地图 → schedule_plan (maze/大图策略)
    schedule_plan(time_limit_ms, proposed_schedule);
}

} // namespace optsvr
