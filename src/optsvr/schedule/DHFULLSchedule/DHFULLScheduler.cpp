// optsvr/schedule/DHFULLSchedule/DHFULLScheduler.cpp
// 完全拷贝 IDEA/DHFULL 的调度逻辑，只做接口对接
// 三个源文件（PrecomputedHeuristicTable.cpp + TaskScheduler.cpp + LargeR + OW）合并

#include "DHFULLScheduler.h"
#include "SharedEnv.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace optsvr {
namespace dhfull_internal {

// ═══════════════════════════════════════════════════════════════════════════════
// PrecomputedHeuristicTable（全量拷贝）
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int kThreadCount = 32;
static constexpr int kBlockedDistance = -1;
static constexpr char kCacheMagic[8] = {'P', 'H', 'E', 'U', 'R', 'V', '1', '\0'};
static constexpr char kDefaultCacheDir[] = "precomputed";
static constexpr size_t kMaxPrecomputeBytes = 20ull * 1024ull * 1024ull * 1024ull;
static constexpr int kMaxAgentsForPrecomputation = 9999;

struct CacheHeader {
    char magic[8];
    int rows;
    int cols;
    long long free_cells;
};

static std::string pht_initialized_map_name;
static std::vector<int> pht_free_cell_id_by_location;
static std::vector<int> pht_location_by_free_cell_id;
static std::vector<std::vector<int>> pht_neighbors_by_free_cell_id;
static std::vector<int16_t> pht_distances_between_cells;
static long long pht_number_of_free_cells = 0;
static bool pht_initialized = false;

static void pht_reset()
{
    pht_distances_between_cells.clear();
    pht_distances_between_cells.shrink_to_fit();
    pht_initialized = false;
    pht_initialized_map_name.clear();
}

static bool pht_get_table_requirements(size_t& table_size, size_t& table_bytes)
{
    const unsigned __int128 free_cells = static_cast<unsigned __int128>(pht_number_of_free_cells);
    const unsigned __int128 entries = free_cells * (free_cells + 1) / 2;
    const unsigned __int128 bytes = entries * sizeof(int16_t);

    if (entries > std::numeric_limits<size_t>::max() ||
        bytes > std::numeric_limits<size_t>::max()) {
        return false;
    }

    table_size = static_cast<size_t>(entries);
    table_bytes = static_cast<size_t>(bytes);
    return true;
}

static size_t pht_triangle_index(long long cell1, long long cell2)
{
    if (cell1 > cell2) std::swap(cell1, cell2);
    return static_cast<size_t>(cell1 * pht_number_of_free_cells - (cell1 * (cell1 - 1) / 2) + cell2 - cell1);
}

static void pht_ensure_cache_directory(SharedEnvironment* env)
{
    if (env->file_storage_path.empty()) {
        env->file_storage_path = kDefaultCacheDir;
    }
    std::error_code ec;
    std::filesystem::create_directories(env->file_storage_path, ec);
}

static std::string pht_cache_path(const SharedEnvironment* env)
{
    if (env->file_storage_path.empty()) return "";
    return env->file_storage_path + "/" + env->map_name + "_dhfull_get_h_v1.bin";
}

static void pht_build_graph(const SharedEnvironment* env)
{
    pht_free_cell_id_by_location.assign(env->map.size(), -1);
    pht_location_by_free_cell_id.clear();
    pht_location_by_free_cell_id.reserve(env->map.size());

    for (int location = 0; location < static_cast<int>(env->map.size()); ++location) {
        if (env->map[location] != 0) continue;
        pht_free_cell_id_by_location[location] = static_cast<int>(pht_location_by_free_cell_id.size());
        pht_location_by_free_cell_id.push_back(location);
    }

    pht_number_of_free_cells = static_cast<long long>(pht_location_by_free_cell_id.size());
    pht_neighbors_by_free_cell_id.assign(static_cast<size_t>(pht_number_of_free_cells), {});

    for (long long free_id = 0; free_id < pht_number_of_free_cells; ++free_id) {
        const int location = pht_location_by_free_cell_id[static_cast<size_t>(free_id)];
        const int row = location / env->cols;
        const int col = location % env->cols;

        const int candidates[4] = {
            (row > 0) ? location - env->cols : -1,
            (row + 1 < env->rows) ? location + env->cols : -1,
            (col > 0) ? location - 1 : -1,
            (col + 1 < env->cols) ? location + 1 : -1,
        };

        for (int next_location : candidates) {
            if (next_location < 0) continue;
            const int next_id = pht_free_cell_id_by_location[next_location];
            if (next_id != -1) {
                pht_neighbors_by_free_cell_id[static_cast<size_t>(free_id)].push_back(next_id);
            }
        }
    }
}

static void pht_write_cache(const SharedEnvironment* env)
{
    const std::string path = pht_cache_path(env);
    if (path.empty()) return;

    std::ofstream out(path, std::ios::binary);
    if (!out.good()) return;

    CacheHeader header{};
    std::memcpy(header.magic, kCacheMagic, sizeof(kCacheMagic));
    header.rows = env->rows;
    header.cols = env->cols;
    header.free_cells = pht_number_of_free_cells;

    out.write(reinterpret_cast<const char*>(&header), sizeof(header));
    out.write(reinterpret_cast<const char*>(pht_distances_between_cells.data()),
              static_cast<std::streamsize>(pht_distances_between_cells.size() * sizeof(int16_t)));
    std::cerr << "[DHFULLSchedule] get_h cache saved to: " << path << "\n";
}

static bool pht_read_cache(const SharedEnvironment* env)
{
    const std::string path = pht_cache_path(env);
    if (path.empty()) return false;

    std::ifstream in(path, std::ios::binary);
    if (!in.good()) return false;

    CacheHeader header{};
    in.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (in.fail()) return false;

    if (std::memcmp(header.magic, kCacheMagic, sizeof(kCacheMagic)) != 0 ||
        header.rows != env->rows ||
        header.cols != env->cols ||
        header.free_cells != pht_number_of_free_cells) {
        return false;
    }

    size_t expected_size = 0;
    size_t expected_bytes = 0;
    if (!pht_get_table_requirements(expected_size, expected_bytes) ||
        expected_bytes > kMaxPrecomputeBytes) {
        return false;
    }

    pht_distances_between_cells.resize(expected_size);
    in.read(reinterpret_cast<char*>(pht_distances_between_cells.data()),
            static_cast<std::streamsize>(pht_distances_between_cells.size() * sizeof(int16_t)));
    if (in.fail()) {
        pht_distances_between_cells.clear();
        return false;
    }

    std::cerr << "[DHFULLSchedule] get_h cache loaded from: " << path << "\n";
    return true;
}

static void pht_compute_all_pairs()
{
    size_t table_size = 0;
    size_t ignored_table_bytes = 0;
    if (!pht_get_table_requirements(table_size, ignored_table_bytes)) {
        throw std::runtime_error("Precomputed heuristic table size overflow");
    }

    pht_distances_between_cells.assign(table_size, static_cast<int16_t>(kBlockedDistance));

#ifdef _OPENMP
#pragma omp parallel num_threads(kThreadCount)
    {
        std::vector<int> dist(static_cast<size_t>(pht_number_of_free_cells), kBlockedDistance);
        std::queue<int> open;

#pragma omp for schedule(dynamic, 1)
        for (long long source_id = 0; source_id < pht_number_of_free_cells; ++source_id) {
            std::fill(dist.begin(), dist.end(), kBlockedDistance);
            while (!open.empty()) open.pop();

            dist[static_cast<size_t>(source_id)] = 0;
            open.push(static_cast<int>(source_id));

            while (!open.empty()) {
                const int current_id = open.front();
                open.pop();
                const int current_dist = dist[static_cast<size_t>(current_id)];

                if (current_id >= source_id) {
                    pht_distances_between_cells[pht_triangle_index(source_id, current_id)] =
                        static_cast<int16_t>(current_dist);
                }

                for (int next_id : pht_neighbors_by_free_cell_id[static_cast<size_t>(current_id)]) {
                    if (dist[static_cast<size_t>(next_id)] != kBlockedDistance) continue;
                    dist[static_cast<size_t>(next_id)] = current_dist + 1;
                    open.push(next_id);
                }
            }
        }
    }
#else
    // 无 OpenMP 时单线程
    std::vector<int> dist(static_cast<size_t>(pht_number_of_free_cells), kBlockedDistance);
    std::queue<int> open;
    for (long long source_id = 0; source_id < pht_number_of_free_cells; ++source_id) {
        std::fill(dist.begin(), dist.end(), kBlockedDistance);
        while (!open.empty()) open.pop();
        dist[static_cast<size_t>(source_id)] = 0;
        open.push(static_cast<int>(source_id));
        while (!open.empty()) {
            const int current_id = open.front();
            open.pop();
            const int current_dist = dist[static_cast<size_t>(current_id)];
            if (current_id >= source_id) {
                pht_distances_between_cells[pht_triangle_index(source_id, current_id)] =
                    static_cast<int16_t>(current_dist);
            }
            for (int next_id : pht_neighbors_by_free_cell_id[static_cast<size_t>(current_id)]) {
                if (dist[static_cast<size_t>(next_id)] != kBlockedDistance) continue;
                dist[static_cast<size_t>(next_id)] = current_dist + 1;
                open.push(next_id);
            }
        }
    }
#endif
}

static void pht_initialize(SharedEnvironment* env)
{
    if (pht_initialized && pht_initialized_map_name == env->map_name) return;

    pht_reset();
    pht_ensure_cache_directory(env);
    pht_build_graph(env);

    if (env->num_of_agents > kMaxAgentsForPrecomputation) {
        std::cerr << "[DHFULLSchedule] Skipping precomputed table: agents=" << env->num_of_agents << "\n";
        pht_initialized_map_name = env->map_name;
        pht_initialized = true;
        return;
    }

    size_t table_size = 0;
    size_t table_bytes = 0;
    const bool has_valid_size = pht_get_table_requirements(table_size, table_bytes);
    if (!has_valid_size || table_bytes > kMaxPrecomputeBytes) {
        std::cerr << "[DHFULLSchedule] Skipping precomputed table: too large\n";
        pht_initialized_map_name = env->map_name;
        pht_initialized = true;
        return;
    }

    if (!pht_read_cache(env)) {
        std::cerr << "[DHFULLSchedule] Computing get_h table (" << kThreadCount << " threads)...\n";
        pht_compute_all_pairs();
        pht_write_cache(env);
    }

    pht_initialized_map_name = env->map_name;
    pht_initialized = true;
}

static bool pht_available()
{
    return pht_initialized && !pht_distances_between_cells.empty();
}

static int pht_get_distance(int source, int target)
{
    if (!pht_available()) return std::numeric_limits<int>::max();
    if (source == target) return 0;
    if (source < 0 || target < 0 ||
        source >= static_cast<int>(pht_free_cell_id_by_location.size()) ||
        target >= static_cast<int>(pht_free_cell_id_by_location.size())) {
        return std::numeric_limits<int>::max();
    }
    const int source_id = pht_free_cell_id_by_location[source];
    const int target_id = pht_free_cell_id_by_location[target];
    if (source_id == -1 || target_id == -1) return std::numeric_limits<int>::max();
    return pht_distances_between_cells[pht_triangle_index(source_id, target_id)];
}

// ═══════════════════════════════════════════════════════════════════════════════
// scheduler_distance / task_path_cost（与原版一致）
// ═══════════════════════════════════════════════════════════════════════════════

static int scheduler_distance(SharedEnvironment* env, int source, int target)
{
    if (pht_available()) return pht_get_distance(source, target);
    return DefaultPlanner::get_h(env, source, target);
}

static int task_path_cost(SharedEnvironment* env, int agent_loc, const Task& task)
{
    if (task.locations.empty()) return 0;
    int total_dist = scheduler_distance(env, agent_loc, task.locations[0]);
    int current_loc = task.locations[0];
    for (size_t i = 1; i < task.locations.size(); ++i) {
        total_dist += scheduler_distance(env, current_loc, task.locations[i]);
        current_loc = task.locations[i];
    }
    return total_dist;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 全局状态（原 DHFULL 使用的全局变量）
// ═══════════════════════════════════════════════════════════════════════════════

static std::mt19937 mt;
static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, int> task_assignments;
static std::unordered_map<int, int> location_congestion;

// Room 部分分配（mode=2 静态排除）
static std::unordered_set<int> permanently_excluded_agents;
static bool room_assign_initialized = false;
static std::mt19937 room_assign_rng(42);

// ═══════════════════════════════════════════════════════════════════════════════
// 辅助函数（原 TaskScheduler.cpp 匿名命名空间）
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int kReclaimSafetyMarginTicks = 1;
static constexpr int K_BEST_TASKS = 5;

static int optimistic_ticks_to_first_loc(SharedEnvironment* env, int agent_id, const Task& task)
{
    if (task.locations.empty() || agent_id < 0 || agent_id >= static_cast<int>(env->system_states.size())) {
        return std::numeric_limits<int>::max();
    }
    const State& state = env->system_states.at(agent_id);
    int optimistic_loc = state.location;
    int remaining_ticks = 0;
    if (state.counter.count > 0 && state.current_action != Action::NA) {
        remaining_ticks = std::max(0, state.counter.maxCount - state.counter.count);
        if (state.current_action == Action::FW) {
            const int moves[4] = {1, env->cols, -1, -env->cols};
            optimistic_loc += moves[state.orientation];
        }
    }
    const int dist = scheduler_distance(env, optimistic_loc, task.locations[0]);
    return remaining_ticks + dist * env->max_counter;
}

static bool can_reclaim_task(SharedEnvironment* env, int agent_id, const Task& task)
{
    if (task.idx_next_loc != 0 || task.locations.empty() || task.agent_assigned != agent_id) return false;
    if (agent_id < 0 || agent_id >= static_cast<int>(env->curr_task_schedule.size()) ||
        env->curr_task_schedule[agent_id] != task.task_id) return false;
    const int action_time = std::max(1, env->action_time);
    int horizon_ticks = (env->min_planner_communication_time + action_time - 1) / action_time;
    horizon_ticks += kReclaimSafetyMarginTicks;
    return optimistic_ticks_to_first_loc(env, agent_id, task) > horizon_ticks;
}

static void rebuild_scheduler_state(SharedEnvironment* env, std::vector<int>& proposed_schedule)
{
    free_agents.clear();
    free_tasks.clear();
    task_assignments.clear();

    for (int agent_id = 0; agent_id < static_cast<int>(env->curr_task_schedule.size()); ++agent_id) {
        if (env->curr_task_schedule[agent_id] == -1) {
            free_agents.insert(agent_id);
            proposed_schedule[agent_id] = -1;
        }
    }

    for (const auto& [task_id, task] : env->task_pool) {
        if (task.agent_assigned == -1) {
            free_tasks.insert(task_id);
        } else {
            task_assignments[task_id] = task.agent_assigned;
        }
    }
}

static void reclaim_safe_tasks(SharedEnvironment* env, std::vector<int>& proposed_schedule)
{
    for (auto it = task_assignments.begin(); it != task_assignments.end(); ) {
        const int task_id = it->first;
        const int agent_id = it->second;
        auto task_it = env->task_pool.find(task_id);
        if (task_it == env->task_pool.end()) {
            it = task_assignments.erase(it);
            continue;
        }
        if (can_reclaim_task(env, agent_id, task_it->second)) {
            free_tasks.insert(task_id);
            free_agents.insert(agent_id);
            proposed_schedule[agent_id] = -1;
            it = task_assignments.erase(it);
        } else {
            ++it;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 匈牙利算法（double 版）
// ═══════════════════════════════════════════════════════════════════════════════

static double hungarian_solve(std::vector<std::vector<double>>& cost_matrix, std::vector<int>& assignment)
{
    int n = cost_matrix.size();
    if (n == 0) return 0.0;
    int m = cost_matrix[0].size();
    if (m == 0) return 0.0;

    bool transpose = n > m;
    if (transpose) {
        std::swap(n, m);
        std::vector<std::vector<double>> temp(m, std::vector<double>(n));
        for (int i = 0; i < m; ++i)
            for (int j = 0; j < n; ++j)
                temp[i][j] = cost_matrix[j][i];
        cost_matrix.swap(temp);
    }

    std::vector<double> u(n+1, 0), v(m+1, 0);
    std::vector<int> p(m+1, 0), way(m+1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<double> minv(m+1, std::numeric_limits<double>::max());
        std::vector<bool> used(m+1, false);
        do {
            used[j0] = true;
            int i0 = p[j0];
            double delta = std::numeric_limits<double>::max();
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    double cur = cost_matrix[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else          { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    assignment.assign(n, -1);
    for (int j = 1; j <= m; ++j)
        if (p[j] != 0) assignment[p[j]-1] = j-1;

    if (transpose) {
        std::vector<int> tmp(m, -1);
        for (int i = 0; i < n; ++i)
            if (assignment[i] != -1) tmp[assignment[i]] = i;
        assignment.swap(tmp);
    }
    return -v[0];
}

// ═══════════════════════════════════════════════════════════════════════════════
// s 图 调度（cluster + Hungarian）
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int MAX_CLUSTER_AGENTS = 5;
static constexpr int MAX_CLUSTER_TASKS  = 10;
static constexpr int MIN_RADIUS = 5;
static constexpr int MAX_RADIUS = 50;
static constexpr int BASE_RADIUS = 20;
static constexpr double DENSITY_LOW_THRESHOLD = 0.5;
static constexpr double DENSITY_HIGH_THRESHOLD = 2.0;

static int computeDynamicRadius(int num_free_agents, int num_free_tasks)
{
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

static int computeMakespanHelper(SharedEnvironment* env, int agent_id, int task_id)
{
    int dist = 0;
    int c_loc = env->curr_states.at(agent_id).location;
    const auto& task = env->task_pool[task_id];
    for (int loc : task.locations) {
        dist += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    return dist;
}

static int hungarianMinMatch(const std::vector<std::vector<int>>& cost, std::vector<int>& assignment)
{
    int n = cost.size();
    int m = cost[0].size();
    assignment.assign(n, -1);
    if (n == 0 || m == 0) return 0;
    if (n > m) return 0;

    std::vector<int> u(n+1), v(m+1), p(m+1), way(m+1);
    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<int> minv(m+1, std::numeric_limits<int>::max());
        std::vector<bool> used(m+1, false);
        do {
            used[j0] = true;
            int i0 = p[j0];
            int delta = std::numeric_limits<int>::max();
            int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (!used[j]) {
                    int cur = cost[i0-1][j-1] - u[i0] - v[j];
                    if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                    if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else          { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (int j = 1; j <= m; ++j) {
        if (p[j] > 0 && p[j] <= n) assignment[p[j]-1] = j-1;
    }
    return -v[0];
}

static void plan_s(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int dynamic_radius = computeDynamicRadius(free_agents.size(), free_tasks.size());
    int extended_radius = dynamic_radius * 2;

    while (!free_agents.empty()) {
        if (std::chrono::steady_clock::now() > endtime) break;

        int seed_agent = *free_agents.begin();
        int seed_loc = env->curr_states[seed_agent].location;

        std::vector<int> cluster_agents;
        std::vector<int> cluster_tasks;
        cluster_agents.push_back(seed_agent);

        std::vector<std::pair<int, int>> task_distances;
        task_distances.reserve(free_tasks.size());
        for (int t_id : free_tasks) {
            int d = DefaultPlanner::get_h(env, seed_loc, env->task_pool[t_id].locations.front());
            task_distances.push_back({d, t_id});
        }
        std::sort(task_distances.begin(), task_distances.end());

        std::unordered_set<int> candidate_task_set;
        int max_candidate_tasks = std::min(static_cast<int>(task_distances.size()), MAX_CLUSTER_TASKS + 5);
        for (int i = 0; i < max_candidate_tasks; ++i) {
            candidate_task_set.insert(task_distances[i].second);
        }

        for (int other_agent : free_agents) {
            if (cluster_agents.size() >= static_cast<size_t>(MAX_CLUSTER_AGENTS)) break;
            if (other_agent == seed_agent) continue;
            int other_loc = env->curr_states[other_agent].location;
            bool is_near = false;
            for (int t_id : candidate_task_set) {
                int d = DefaultPlanner::get_h(env, other_loc, env->task_pool[t_id].locations.front());
                if (d < dynamic_radius) { is_near = true; break; }
                if (t_id % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
            }
            if (is_near) cluster_agents.push_back(other_agent);
        }

        for (int t_id : free_tasks) {
            if (cluster_tasks.size() >= static_cast<size_t>(MAX_CLUSTER_TASKS)) break;
            int min_dist_to_cluster = std::numeric_limits<int>::max();
            for (int a_id : cluster_agents) {
                int d = DefaultPlanner::get_h(env, env->curr_states[a_id].location,
                    env->task_pool[t_id].locations.front());
                min_dist_to_cluster = std::min(min_dist_to_cluster, d);
                if (min_dist_to_cluster < dynamic_radius) break;
            }
            if (min_dist_to_cluster < extended_radius) cluster_tasks.push_back(t_id);
        }

        if (cluster_agents.size() <= 1 || cluster_tasks.size() <= 1) {
            int best_task = -1;
            int min_makespan = std::numeric_limits<int>::max();
            int count = 0;
            for (int t_id : free_tasks) {
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                int makespan = computeMakespanHelper(env, seed_agent, t_id);
                if (makespan < min_makespan) { min_makespan = makespan; best_task = t_id; }
                count++;
            }
            if (best_task != -1) {
                proposed_schedule[seed_agent] = best_task;
                free_agents.erase(seed_agent);
                free_tasks.erase(best_task);
            } else {
                proposed_schedule[seed_agent] = -1;
                free_agents.erase(seed_agent);
            }
        } else {
            size_t n_agents = cluster_agents.size();
            size_t n_tasks = cluster_tasks.size();
            std::vector<std::vector<int>> cost_matrix(n_agents, std::vector<int>(n_tasks));
            for (size_t i = 0; i < n_agents; ++i)
                for (size_t j = 0; j < n_tasks; ++j)
                    cost_matrix[i][j] = computeMakespanHelper(env, cluster_agents[i], cluster_tasks[j]);

            std::vector<int> assignment;
            hungarianMinMatch(cost_matrix, assignment);

            for (size_t i = 0; i < n_agents; ++i) {
                int agent_id = cluster_agents[i];
                if (i < assignment.size() && assignment[i] >= 0 && static_cast<size_t>(assignment[i]) < n_tasks) {
                    int task_id = cluster_tasks[assignment[i]];
                    proposed_schedule[agent_id] = task_id;
                    free_tasks.erase(task_id);
                } else {
                    proposed_schedule[agent_id] = -1;
                }
                free_agents.erase(agent_id);
            }
        }
    }

    for (int agent_id : free_agents) proposed_schedule[agent_id] = -1;
    free_agents.clear();
}

// ═══════════════════════════════════════════════════════════════════════════════
// r 图 < 300 agents（bidding/auction）
// ═══════════════════════════════════════════════════════════════════════════════

static void plan_small_r(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    for (auto it = free_agents.begin(); it != free_agents.end(); ) {
        const int agent_id = *it;
        if (agent_id < 0 || agent_id >= static_cast<int>(env->curr_task_schedule.size()) ||
            env->curr_task_schedule[agent_id] != -1) {
            it = free_agents.erase(it);
        } else {
            proposed_schedule[agent_id] = -1;
            ++it;
        }
    }
    for (auto it = free_tasks.begin(); it != free_tasks.end(); ) {
        auto task_it = env->task_pool.find(*it);
        if (task_it == env->task_pool.end() || task_it->second.agent_assigned != -1 ||
            task_it->second.locations.empty()) {
            it = free_tasks.erase(it);
        } else {
            ++it;
        }
    }
    if (free_agents.empty() || free_tasks.empty()) return;

    // ═══════════════ Room 部分分配 ═══════════════
    // 被排除的 agent 暂存到 deferred_agents，匹配完成后再放回 free_agents
    std::vector<int> deferred_agents;
    const int assign_mode = cfg().room_assign_agent_mode;
    const float assign_ratio = cfg().room_assign_ratio;
    if (assign_mode > 0 && assign_ratio < 0.999f && free_agents.size() > 1) {
        int total_agents = env->num_of_agents;
        int na = (int)free_agents.size();
        int already_busy = total_agents - na;
        int max_active = std::max(1, (int)(assign_ratio * total_agents));
        int max_new = std::max(0, max_active - already_busy);

        if (assign_mode == 2) {
            if (!room_assign_initialized) {
                room_assign_initialized = true;
                permanently_excluded_agents.clear();
                int num_exclude = total_agents - max_active;
                if (num_exclude > 0) {
                    std::vector<int> all_ids(total_agents);
                    std::iota(all_ids.begin(), all_ids.end(), 0);
                    std::shuffle(all_ids.begin(), all_ids.end(), room_assign_rng);
                    for (int i = 0; i < num_exclude; ++i)
                        permanently_excluded_agents.insert(all_ids[i]);
                }
                std::cerr << "[DHFULLSchedule] mode=2 static: excluded "
                          << permanently_excluded_agents.size() << "/" << total_agents << " agents\n";
            }
            for (auto it = free_agents.begin(); it != free_agents.end(); ) {
                if (permanently_excluded_agents.count(*it)) {
                    deferred_agents.push_back(*it);
                    proposed_schedule[*it] = -1;
                    it = free_agents.erase(it);
                } else { ++it; }
            }
        } else if (assign_mode == 1 && na > max_new && max_new >= 0) {
            const int topk = cfg().room_topk_tasks;
            std::vector<std::pair<double, int>> scored;
            scored.reserve(free_agents.size());
            std::vector<int> task_vec(free_tasks.begin(), free_tasks.end());
            int nt = (int)task_vec.size();
            for (int aid : free_agents) {
                if (std::chrono::steady_clock::now() > endtime) {
                    scored.emplace_back(0.0, aid);
                    continue;
                }
                int aloc = env->curr_states.at(aid).location;
                std::vector<int> dists;
                dists.reserve(nt);
                for (int tid : task_vec) {
                    int tloc = env->task_pool.at(tid).locations[0];
                    dists.push_back(DefaultPlanner::get_h(env, aloc, tloc));
                }
                int k = std::min(topk, (int)dists.size());
                std::partial_sort(dists.begin(), dists.begin() + k, dists.end());
                double sum = 0;
                for (int i = 0; i < k; ++i) sum += dists[i];
                scored.emplace_back(sum, aid);
            }
            std::sort(scored.begin(), scored.end());
            std::unordered_set<int> keep_set;
            for (int i = 0; i < std::min(max_new, (int)scored.size()); ++i)
                keep_set.insert(scored[i].second);
            for (auto it = free_agents.begin(); it != free_agents.end(); ) {
                if (!keep_set.count(*it)) {
                    deferred_agents.push_back(*it);
                    proposed_schedule[*it] = -1;
                    it = free_agents.erase(it);
                } else { ++it; }
            }
        }
        if (free_agents.empty()) {
            // 被排除的 agent 放回 free_agents，下轮重新参与筛选
            for (int a : deferred_agents) free_agents.insert(a);
            return;
        }
    }
    // ═══════════════ 部分分配结束 ═══════════════

    struct AgentBid {
        int agent_id;
        std::vector<std::pair<int, int>> top_k;
        int curr_idx;
        explicit AgentBid(int aid) : agent_id(aid), curr_idx(0) { top_k.reserve(K_BEST_TASKS); }
        int get_best_available(const std::unordered_set<int>& occupied) {
            while (curr_idx < static_cast<int>(top_k.size())) {
                if (occupied.find(top_k[curr_idx].first) == occupied.end()) return top_k[curr_idx].first;
                curr_idx++;
            }
            return -1;
        }
        int get_best_cost(const std::unordered_set<int>& occupied) {
            int task = get_best_available(occupied);
            if (task == -1) return INT_MAX;
            return top_k[curr_idx].second;
        }
        int yield_cost(const std::unordered_set<int>& occupied) {
            int saved_idx = curr_idx;
            int old_cost = get_best_cost(occupied);
            curr_idx++;
            int new_cost = get_best_cost(occupied);
            curr_idx = saved_idx;
            if (new_cost == INT_MAX) return INT_MAX;
            return new_cost - old_cost;
        }
    };

    std::vector<AgentBid> bids;
    bids.reserve(free_agents.size());
    int eval_count = 0;

    for (int agent_id : free_agents) {
        if (eval_count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
        AgentBid bid(agent_id);
        int start_loc = env->curr_states.at(agent_id).location;
        for (int task_id : free_tasks) {
            if (eval_count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
            auto task_it = env->task_pool.find(task_id);
            if (task_it == env->task_pool.end() || task_it->second.locations.empty()) continue;
            int dist = 0;
            int curr_loc = start_loc;
            for (int loc : task_it->second.locations) {
                dist += DefaultPlanner::get_h(env, curr_loc, loc);
                curr_loc = loc;
            }
            if (bid.top_k.size() < K_BEST_TASKS) {
                auto it2 = bid.top_k.begin();
                while (it2 != bid.top_k.end() && it2->second < dist) ++it2;
                bid.top_k.insert(it2, {task_id, dist});
            } else if (dist < bid.top_k.back().second) {
                auto it2 = bid.top_k.begin();
                while (it2 != bid.top_k.end() && it2->second < dist) ++it2;
                bid.top_k.insert(it2, {task_id, dist});
                bid.top_k.pop_back();
            }
            eval_count++;
        }
        if (!bid.top_k.empty()) bids.push_back(bid);
    }
    if (bids.empty()) return;

    std::vector<int> agent_assignment(bids.size(), -1);
    std::unordered_map<int, std::vector<int>> task_claimants;
    std::unordered_set<int> occupied;

    for (int i = 0; i < static_cast<int>(bids.size()); ++i) {
        int task = bids[i].get_best_available(occupied);
        if (task != -1) {
            agent_assignment[i] = task;
            task_claimants[task].push_back(i);
            occupied.insert(task);
        }
    }

    const int MAX_ITER = static_cast<int>(bids.size()) * 3;
    int iteration = 0;
    while (iteration < MAX_ITER && std::chrono::steady_clock::now() < endtime) {
        std::vector<int> conflicted;
        for (auto& entry : task_claimants) {
            if (entry.second.size() > 1) conflicted.push_back(entry.first);
        }
        if (conflicted.empty()) break;
        iteration++;
        for (int task_id : conflicted) {
            auto& claimants = task_claimants[task_id];
            if (claimants.size() <= 1) continue;
            int min_yield = INT_MAX, yield_idx = -1;
            for (int idx : claimants) {
                std::unordered_set<int> current_occupied;
                for (int at : agent_assignment) {
                    if (at != -1) current_occupied.insert(at);
                }
                int ycost = bids[idx].yield_cost(current_occupied);
                if (ycost < min_yield) { min_yield = ycost; yield_idx = idx; }
            }
            if (yield_idx == -1) continue;
            auto& old_claimants = task_claimants[task_id];
            for (auto it = old_claimants.begin(); it != old_claimants.end(); ++it) {
                if (*it == yield_idx) { old_claimants.erase(it); break; }
            }
            if (old_claimants.empty()) task_claimants.erase(task_id);
            std::unordered_set<int> remaining_occupied;
            for (auto& entry : task_claimants) {
                remaining_occupied.insert(entry.first);
            }
            int new_task = bids[yield_idx].get_best_available(remaining_occupied);
            if (new_task != -1) {
                agent_assignment[yield_idx] = new_task;
                task_claimants[new_task].push_back(yield_idx);
            } else {
                agent_assignment[yield_idx] = -1;
            }
        }
    }

    for (int i = 0; i < static_cast<int>(bids.size()); ++i) {
        int assigned_task = agent_assignment[i];
        if (assigned_task != -1 && env->task_pool.find(assigned_task) != env->task_pool.end()) {
            proposed_schedule[bids[i].agent_id] = assigned_task;
            free_agents.erase(bids[i].agent_id);
            free_tasks.erase(assigned_task);
        } else {
            // 未匹配到任务的 agent 保留在 free_agents 中，下轮继续参与竞标
            proposed_schedule[bids[i].agent_id] = -1;
        }
    }

    // 被部分分配排除的 agent 放回 free_agents，下轮重新参与筛选
    for (int a : deferred_agents) free_agents.insert(a);
}

// ═══════════════════════════════════════════════════════════════════════════════
// r 图 > 300 agents (K-means)
// ═══════════════════════════════════════════════════════════════════════════════

struct Cluster {
    std::vector<int> task_ids;
    std::pair<int, int> center;
};

static std::vector<Cluster> clusters;
static std::unordered_map<int, int> task_to_cluster;
static int last_clustered_free_tasks_count = 0;
static std::mt19937 rng(42);

static std::pair<int, int> get_task_location(SharedEnvironment* env, int task_id)
{
    if (env->task_pool.find(task_id) != env->task_pool.end() &&
        !env->task_pool[task_id].locations.empty()) {
        int loc = env->task_pool[task_id].locations[0];
        return {loc / env->cols, loc % env->cols};
    }
    return {0, 0};
}

static double euclidean_distance(const std::pair<int, int>& a, const std::pair<int, int>& b)
{
    double dr = a.first - b.first;
    double dc = a.second - b.second;
    return std::sqrt(dr * dr + dc * dc);
}

static void cluster_tasks(SharedEnvironment* env)
{
    if (free_tasks.empty()) { clusters.clear(); task_to_cluster.clear(); return; }

    int K = std::max(1, static_cast<int>(env->num_of_agents / 2));
    K = std::min(K, static_cast<int>(free_tasks.size()));

    std::vector<std::pair<int, int>> task_positions;
    std::vector<int> task_ids;
    for (int t_id : free_tasks) {
        task_positions.push_back(get_task_location(env, t_id));
        task_ids.push_back(t_id);
    }

    clusters.clear();
    clusters.resize(K);
    std::vector<int> indices(task_ids.size());
    for (int i = 0; i < static_cast<int>(indices.size()); i++) indices[i] = i;
    std::shuffle(indices.begin(), indices.end(), rng);
    for (int i = 0; i < K; i++) clusters[i].center = task_positions[indices[i]];

    const int max_iterations = 10;
    for (int iter = 0; iter < max_iterations; iter++) {
        for (auto& cluster : clusters) cluster.task_ids.clear();
        for (int i = 0; i < static_cast<int>(task_ids.size()); i++) {
            int best_cluster = 0;
            double min_dist = std::numeric_limits<double>::max();
            for (int c = 0; c < K; c++) {
                double dist = euclidean_distance(task_positions[i], clusters[c].center);
                if (dist < min_dist) { min_dist = dist; best_cluster = c; }
            }
            clusters[best_cluster].task_ids.push_back(task_ids[i]);
            task_to_cluster[task_ids[i]] = best_cluster;
        }
        bool converged = true;
        for (int c = 0; c < K; c++) {
            if (clusters[c].task_ids.empty()) continue;
            double sum_row = 0, sum_col = 0;
            for (int t_id : clusters[c].task_ids) {
                auto pos = get_task_location(env, t_id);
                sum_row += pos.first; sum_col += pos.second;
            }
            std::pair<int, int> new_center = {
                static_cast<int>(sum_row / clusters[c].task_ids.size()),
                static_cast<int>(sum_col / clusters[c].task_ids.size())
            };
            if (new_center != clusters[c].center) { converged = false; clusters[c].center = new_center; }
        }
        if (converged) break;
    }
    last_clustered_free_tasks_count = free_tasks.size();
}

static int find_nearest_cluster(SharedEnvironment* env, int agent_id)
{
    int agent_loc = env->curr_states.at(agent_id).location;
    std::pair<int, int> agent_pos = {agent_loc / env->cols, agent_loc % env->cols};
    int best_cluster = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int c = 0; c < static_cast<int>(clusters.size()); c++) {
        if (clusters[c].task_ids.empty()) continue;
        double dist = euclidean_distance(agent_pos, clusters[c].center);
        if (dist < min_dist) { min_dist = dist; best_cluster = c; }
    }
    return best_cluster;
}

static void plan_large_r(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // ═══════════════ Room 部分分配 ═══════════════
    // 被排除的 agent 暂存到 deferred_agents，匹配完成后再放回 free_agents
    std::vector<int> deferred_agents;
    const int assign_mode = cfg().room_assign_agent_mode;
    const float assign_ratio = cfg().room_assign_ratio;
    if (assign_mode > 0 && assign_ratio < 0.999f && free_agents.size() > 1) {
        int total_agents = env->num_of_agents;
        int na = (int)free_agents.size();
        int already_busy = total_agents - na;
        int max_active = std::max(1, (int)(assign_ratio * total_agents));
        int max_new = std::max(0, max_active - already_busy);

        if (assign_mode == 2) {
            if (!room_assign_initialized) {
                room_assign_initialized = true;
                permanently_excluded_agents.clear();
                int num_exclude = total_agents - max_active;
                if (num_exclude > 0) {
                    std::vector<int> all_ids(total_agents);
                    std::iota(all_ids.begin(), all_ids.end(), 0);
                    std::shuffle(all_ids.begin(), all_ids.end(), room_assign_rng);
                    for (int i = 0; i < num_exclude; ++i)
                        permanently_excluded_agents.insert(all_ids[i]);
                }
                std::cerr << "[DHFULLSchedule] mode=2 static: excluded "
                          << permanently_excluded_agents.size() << "/" << total_agents << " agents\n";
            }
            for (auto it = free_agents.begin(); it != free_agents.end(); ) {
                if (permanently_excluded_agents.count(*it)) {
                    deferred_agents.push_back(*it);
                    proposed_schedule[*it] = -1;
                    it = free_agents.erase(it);
                } else { ++it; }
            }
        } else if (assign_mode == 1 && na > max_new && max_new >= 0) {
            const int topk = cfg().room_topk_tasks;
            std::vector<std::pair<double, int>> scored;
            scored.reserve(free_agents.size());
            std::vector<int> task_vec(free_tasks.begin(), free_tasks.end());
            int nt = (int)task_vec.size();
            for (int aid : free_agents) {
                if (std::chrono::steady_clock::now() > endtime) {
                    scored.emplace_back(0.0, aid);
                    continue;
                }
                int aloc = env->curr_states.at(aid).location;
                std::vector<int> dists;
                dists.reserve(nt);
                for (int tid : task_vec) {
                    int tloc = env->task_pool.at(tid).locations[0];
                    dists.push_back(DefaultPlanner::get_h(env, aloc, tloc));
                }
                int k = std::min(topk, (int)dists.size());
                std::partial_sort(dists.begin(), dists.begin() + k, dists.end());
                double sum = 0;
                for (int i = 0; i < k; ++i) sum += dists[i];
                scored.emplace_back(sum, aid);
            }
            std::sort(scored.begin(), scored.end());
            std::unordered_set<int> keep_set;
            for (int i = 0; i < std::min(max_new, (int)scored.size()); ++i)
                keep_set.insert(scored[i].second);
            for (auto it = free_agents.begin(); it != free_agents.end(); ) {
                if (!keep_set.count(*it)) {
                    deferred_agents.push_back(*it);
                    proposed_schedule[*it] = -1;
                    it = free_agents.erase(it);
                } else { ++it; }
            }
        }
        if (free_agents.empty()) {
            // 被排除的 agent 放回 free_agents，下轮重新参与筛选
            for (int a : deferred_agents) free_agents.insert(a);
            return;
        }
    }
    // ═══════════════ 部分分配结束 ═══════════════

    if (free_tasks.size() != static_cast<size_t>(last_clustered_free_tasks_count)) {
        cluster_tasks(env);
    }

    auto it = free_agents.begin();
    while (it != free_agents.end()) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int i = *it;
        int min_task_i = -1, min_task_makespan = INT_MAX, count = 0;

        if (!clusters.empty()) {
            int nearest_cluster = find_nearest_cluster(env, i);
            for (int t_id : clusters[nearest_cluster].task_ids) {
                if (free_tasks.find(t_id) == free_tasks.end()) continue;
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                int dist = 0, c_loc = env->curr_states.at(i).location;
                for (int loc : env->task_pool[t_id].locations) {
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
        }

        if (min_task_i == -1) {
            for (int t_id : free_tasks) {
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                int dist = 0, c_loc = env->curr_states.at(i).location;
                for (int loc : env->task_pool[t_id].locations) {
                    dist += DefaultPlanner::get_h(env, c_loc, loc);
                    c_loc = loc;
                }
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
        }

        if (min_task_i != -1) {
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            if (task_to_cluster.find(min_task_i) != task_to_cluster.end()) {
                int cluster_id = task_to_cluster[min_task_i];
                auto& cl = clusters[cluster_id];
                auto it_c = std::find(cl.task_ids.begin(), cl.task_ids.end(), min_task_i);
                if (it_c != cl.task_ids.end()) cl.task_ids.erase(it_c);
                task_to_cluster.erase(min_task_i);
            }
        } else {
            proposed_schedule[i] = -1;
            it++;
        }
    }

    // 被部分分配排除的 agent 放回 free_agents，下轮重新参与筛选
    for (int a : deferred_agents) free_agents.insert(a);
}

// ═══════════════════════════════════════════════════════════════════════════════
// o/w 图 (QuadTree)
// ═══════════════════════════════════════════════════════════════════════════════

struct QuadTreeNode {
    int x_min, y_min, x_max, y_max;
    int level;
    std::vector<int> agent_indices;
    std::vector<int> task_ids;
    QuadTreeNode* children[4];
    bool is_leaf;
    QuadTreeNode(int x1, int y1, int x2, int y2, int lvl)
        : x_min(x1), y_min(y1), x_max(x2), y_max(y2), level(lvl), is_leaf(true) {
        children[0] = children[1] = children[2] = children[3] = nullptr;
    }
    ~QuadTreeNode() { for (int i = 0; i < 4; ++i) delete children[i]; }
    bool contains(int loc, int cols) const {
        int x = loc % cols; int y = loc / cols;
        return x >= x_min && x < x_max && y >= y_min && y < y_max;
    }
    int getChildIndex(int loc, int cols) const {
        int x = loc % cols; int y = loc / cols;
        int mid_x = (x_min + x_max) / 2; int mid_y = (y_min + y_max) / 2;
        if (y < mid_y) return (x < mid_x) ? 0 : 1;
        return (x < mid_x) ? 2 : 3;
    }
    void subdivide(int cols) {
        if (!is_leaf) return;
        is_leaf = false;
        int mid_x = (x_min + x_max) / 2; int mid_y = (y_min + y_max) / 2;
        children[0] = new QuadTreeNode(x_min, y_min, mid_x, mid_y, level + 1);
        children[1] = new QuadTreeNode(mid_x, y_min, x_max, mid_y, level + 1);
        children[2] = new QuadTreeNode(x_min, mid_y, mid_x, y_max, level + 1);
        children[3] = new QuadTreeNode(mid_x, mid_y, x_max, y_max, level + 1);
    }
};

static QuadTreeNode* quad_root = nullptr;
static int ow_map_cols = 100;
static int ow_map_rows = 100;
static constexpr int ow_max_depth = 6;
static constexpr int ow_max_items_per_leaf = 8;

static void ow_initQuadTree(SharedEnvironment* env)
{
    ow_map_cols = env->cols > 0 ? env->cols : 100;
    ow_map_rows = env->rows > 0 ? env->rows : 100;
    if (quad_root) { delete quad_root; quad_root = nullptr; }
    quad_root = new QuadTreeNode(0, 0, ow_map_cols, ow_map_rows, 0);
}

static void ow_insertAgent(SharedEnvironment* env, int agent_id, QuadTreeNode* node)
{
    int loc = env->curr_states.at(agent_id).location;
    if (!node->contains(loc, ow_map_cols)) return;
    if (node->is_leaf) {
        node->agent_indices.push_back(agent_id);
        int cur_items = (int)node->agent_indices.size() + (int)node->task_ids.size();
        if (node->level < ow_max_depth && cur_items > ow_max_items_per_leaf) {
            std::vector<int> ab = node->agent_indices;
            std::vector<int> tb = node->task_ids;
            node->agent_indices.clear(); node->task_ids.clear();
            node->subdivide(ow_map_cols);
            for (int a : ab) {
                int l = env->curr_states.at(a).location;
                int idx = node->getChildIndex(l, ow_map_cols);
                if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(a);
            }
            for (int t : tb) {
                if (!env->task_pool.count(t)) continue;
                int lt = env->task_pool.at(t).locations.front();
                int idx = node->getChildIndex(lt, ow_map_cols);
                if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(t);
            }
        }
    } else {
        int idx = node->getChildIndex(loc, ow_map_cols);
        if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(agent_id);
    }
}

static void ow_insertTask(SharedEnvironment* env, int task_id, QuadTreeNode* node)
{
    if (!env->task_pool.count(task_id)) return;
    int loc = env->task_pool.at(task_id).locations.front();
    if (!node->contains(loc, ow_map_cols)) return;
    if (node->is_leaf) {
        node->task_ids.push_back(task_id);
        int cur_items = (int)node->agent_indices.size() + (int)node->task_ids.size();
        if (node->level < ow_max_depth && cur_items > ow_max_items_per_leaf) {
            std::vector<int> ab = node->agent_indices;
            std::vector<int> tb = node->task_ids;
            node->agent_indices.clear(); node->task_ids.clear();
            node->subdivide(ow_map_cols);
            for (int a : ab) {
                int l = env->curr_states.at(a).location;
                int idx = node->getChildIndex(l, ow_map_cols);
                if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->agent_indices.push_back(a);
            }
            for (int t : tb) {
                if (!env->task_pool.count(t)) continue;
                int lt = env->task_pool.at(t).locations.front();
                int idx = node->getChildIndex(lt, ow_map_cols);
                if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(t);
            }
        }
    } else {
        int idx = node->getChildIndex(loc, ow_map_cols);
        if (idx >= 0 && idx < 4 && node->children[idx]) node->children[idx]->task_ids.push_back(task_id);
    }
}

static void ow_rebuildQuadTree(SharedEnvironment* env)
{
    if (quad_root) { delete quad_root; quad_root = nullptr; }
    quad_root = new QuadTreeNode(0, 0, ow_map_cols, ow_map_rows, 0);
    for (int agent_id : free_agents) ow_insertAgent(env, agent_id, quad_root);
    for (int task_id : free_tasks)   ow_insertTask(env, task_id, quad_root);
}

static void ow_collectTasksInRadius(SharedEnvironment* env, int agent_loc, int radius, std::vector<int>& nearby_tasks)
{
    nearby_tasks.clear();
    int ax = agent_loc % ow_map_cols;
    int ay = agent_loc / ow_map_cols;
    for (int task_id : free_tasks) {
        if (!env->task_pool.count(task_id)) continue;
        int tloc = env->task_pool.at(task_id).locations.front();
        int tx = tloc % ow_map_cols;
        int ty = tloc / ow_map_cols;
        if (abs(tx - ax) + abs(ty - ay) <= radius) nearby_tasks.push_back(task_id);
    }
}

static int ow_estimateTaskMakespan(SharedEnvironment* env, int agent_id, int task_id)
{
    if (!env->task_pool.count(task_id)) return INT_MAX;
    int dist = 0, c_loc = env->curr_states.at(agent_id).location;
    for (int loc : env->task_pool.at(task_id).locations) {
        dist += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    return dist;
}

static void plan_ow(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    ow_rebuildQuadTree(env);

    int search_radius = 30;
    std::vector<int> agent_list(free_agents.begin(), free_agents.end());
    std::shuffle(agent_list.begin(), agent_list.end(), rng);

    for (int i : agent_list) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (env->curr_task_schedule[i] != -1) continue;
        int agent_loc = env->curr_states.at(i).location;
        int best_task = -1, best_makespan = INT_MAX;
        std::vector<int> candidate_tasks;
        ow_collectTasksInRadius(env, agent_loc, search_radius, candidate_tasks);
        if (candidate_tasks.empty()) candidate_tasks.assign(free_tasks.begin(), free_tasks.end());
        int eval_count = 0, max_eval = 200;
        for (int t_id : candidate_tasks) {
            if (!env->task_pool.count(t_id)) continue;
            if (eval_count++ > max_eval) break;
            if (std::chrono::steady_clock::now() > endtime) break;
            int makespan = ow_estimateTaskMakespan(env, i, t_id);
            if (makespan < best_makespan) { best_makespan = makespan; best_task = t_id; }
        }
        if (best_task != -1) {
            proposed_schedule[i] = best_task;
            free_agents.erase(i);
            free_tasks.erase(best_task);
        } else {
            proposed_schedule[i] = -1;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 默认分支 schedule_plan（非 s/r/o/w 图，即 maze 等）
// ═══════════════════════════════════════════════════════════════════════════════

static void schedule_plan(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    rebuild_scheduler_state(env, proposed_schedule);

    // 分支 1：r 图
    if (env->num_of_agents < 500) {
        const int CELL_SIZE_R = 2;
        if (free_agents.size() < 300) {
            reclaim_safe_tasks(env, proposed_schedule);
            std::vector<int> agents(free_agents.begin(), free_agents.end());
            std::vector<int> tasks(free_tasks.begin(), free_tasks.end());
            int num_agents = agents.size(), num_tasks = tasks.size();
            if (num_agents == 0 || num_tasks == 0) {
                for (int aid : free_agents) proposed_schedule[aid] = -1;
                return;
            }
            std::vector<std::vector<double>> cost_matrix(num_agents, std::vector<double>(num_tasks, 0.0));
            for (int i = 0; i < num_agents; ++i) {
                if (std::chrono::steady_clock::now() > endtime) break;
                int agent_id = agents[i];
                for (int j = 0; j < num_tasks; ++j) {
                    if (std::chrono::steady_clock::now() > endtime) break;
                    int task_id = tasks[j];
                    if (!env->task_pool.count(task_id)) continue;
                    const Task& task = env->task_pool.at(task_id);
                    int agent_loc = env->curr_states.at(agent_id).location;
                    double total_dist = scheduler_distance(env, agent_loc, task.locations[0]) * 6;
                    int current_loc = agent_loc;
                    int path_dist = 0;
                    for (const auto& step : task.locations) {
                        path_dist += scheduler_distance(env, current_loc, step);
                        current_loc = step;
                    }
                    total_dist += path_dist;
                    int start_loc = task.locations[0];
                    int end_loc   = task.locations[task.locations.size() - 1];
                    int grid_x  = (start_loc / env->cols) / CELL_SIZE_R;
                    int grid_y  = (start_loc % env->cols) / CELL_SIZE_R;
                    int egrid_x = (end_loc   / env->cols) / CELL_SIZE_R;
                    int egrid_y = (end_loc   % env->cols) / CELL_SIZE_R;
                    int density_penalty = 0, density_penaltyend = 0;
                    for (const auto& other : env->task_pool) {
                        if (other.first == task_id) continue;
                        int os = other.second.locations[0];
                        int ox = (os / env->cols) / CELL_SIZE_R;
                        int oy = (os % env->cols) / CELL_SIZE_R;
                        if (abs(ox - grid_x)  <= 1 && abs(oy - grid_y)  <= 1) density_penalty++;
                        if (abs(ox - egrid_x) <= 1 && abs(oy - egrid_y) <= 1) density_penaltyend++;
                    }
                    double score = -(0.9 * total_dist - 0.1 * density_penalty + 0.2 * density_penaltyend);
                    cost_matrix[i][j] = -score;
                }
            }
            std::vector<int> assignment;
            hungarian_solve(cost_matrix, assignment);
            for (int i = 0; i < num_agents; ++i) {
                if (assignment[i] != -1 && assignment[i] < num_tasks) {
                    int agent_id = agents[i], task_id = tasks[assignment[i]];
                    if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
                        task_assignments[task_id] = agent_id;
                        proposed_schedule[agent_id] = task_id;
                        free_agents.erase(agent_id);
                        free_tasks.erase(task_id);
                    }
                }
            }
            for (int aid : free_agents) proposed_schedule[aid] = -1;
        } else {
            using TaskAssignment = std::tuple<int, int, int>;
            std::priority_queue<TaskAssignment, std::vector<TaskAssignment>, std::greater<>> assignment_queue;
            for (int agent_id : free_agents) {
                for (int task_id : free_tasks) {
                    if (std::chrono::steady_clock::now() > endtime) break;
                    int dist = task_path_cost(env, env->curr_states.at(agent_id).location,
                                              env->task_pool[task_id]) + 5 * scheduler_distance(
                                                  env, env->curr_states.at(agent_id).location,
                                                  env->task_pool[task_id].locations[0]);
                    assignment_queue.emplace(dist, agent_id, task_id);
                }
            }
            while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
                auto [makespan, agent_id, task_id] = assignment_queue.top();
                assignment_queue.pop();
                if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
                    task_assignments[task_id] = agent_id;
                    proposed_schedule[agent_id] = task_id;
                    free_agents.erase(agent_id);
                    free_tasks.erase(task_id);
                }
            }
            for (int aid : free_agents) proposed_schedule[aid] = -1;
        }
        return;
    }

    // 分支 2：大图
    if (free_agents.size() < 110) {
        const double congestion_penalty = 0.1;
        const size_t num_free_agents = free_agents.size();
        const size_t num_free_tasks  = free_tasks.size();
        const double ratio     = num_free_tasks > 0 ? (double)num_free_agents / num_free_tasks : 1.0;
        const double midpoint  = 1.0, steepness = 5.0;
        const double alpha     = 1.0 / (1.0 + exp(-steepness * (ratio - midpoint)));
        const double weight_dist    = 0.5 * (1.0 - alpha);
        const double weight_cluster = 0.2 * alpha;

        using TaskAssignment = std::tuple<double, int, int>;
        std::priority_queue<TaskAssignment> assignment_queue;
        int count = 0;

        for (int agent_id : free_agents) {
            if (std::chrono::steady_clock::now() > endtime) break;
            for (int task_id : free_tasks) {
                if (count % 20 == 0 && std::chrono::steady_clock::now() > endtime) break;
                if (!env->task_pool.count(task_id)) continue;
                const Task& task = env->task_pool.at(task_id);
                int agent_loc = env->curr_states.at(agent_id).location;
                double total_dist = scheduler_distance(env, agent_loc, task.locations[0]) * 10;
                agent_loc = task.locations[0];
                for (size_t i = 1; i < task.locations.size(); i++)
                    total_dist += scheduler_distance(env, agent_loc, task.locations[i]);
                int current_loc = env->curr_states.at(agent_id).location;
                total_dist += congestion_penalty * location_congestion[current_loc];
                for (const auto& loc : task.locations)
                    total_dist += congestion_penalty * location_congestion[loc];
                double cluster_factor = 0; int cluster_count = 0;
                for (const auto& other : env->task_pool) {
                    if (other.first == task_id) continue;
                    double min_dist = std::numeric_limits<double>::max();
                    for (const auto& loc1 : task.locations) {
                        double d = scheduler_distance(env, loc1, other.second.locations[0]);
                        if (d < min_dist) min_dist = d;
                    }
                    cluster_factor += min_dist; cluster_count++;
                }
                cluster_factor = cluster_count > 0 ? cluster_factor / cluster_count : 0;
                double score = -(weight_dist * total_dist - weight_cluster * cluster_factor);
                assignment_queue.emplace(score, agent_id, task_id);
                count++;
            }
        }
        while (!assignment_queue.empty()) {
            auto [score, agent_id, task_id] = assignment_queue.top();
            assignment_queue.pop();
            if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
                task_assignments[task_id] = agent_id;
                proposed_schedule[agent_id] = task_id;
                free_agents.erase(agent_id);
                free_tasks.erase(task_id);
                int current_loc = env->curr_states[agent_id].location;
                location_congestion[current_loc]++;
                for (const auto& loc : env->task_pool[task_id].locations)
                    location_congestion[loc]++;
            }
        }
        for (int aid : free_agents) proposed_schedule[aid] = -1;
    } else if (free_agents.size() < 1000) {
        using TaskAssignment = std::tuple<int, int, int>;
        std::priority_queue<TaskAssignment, std::vector<TaskAssignment>, std::greater<>> assignment_queue;
        for (int agent_id : free_agents) {
            for (int task_id : free_tasks) {
                if (std::chrono::steady_clock::now() > endtime) break;
                int dist = task_path_cost(env, env->curr_states.at(agent_id).location,
                                          env->task_pool[task_id]) + 7 * scheduler_distance(
                                              env, env->curr_states.at(agent_id).location,
                                              env->task_pool[task_id].locations[0]);
                assignment_queue.emplace(dist, agent_id, task_id);
            }
        }
        while (!assignment_queue.empty() && std::chrono::steady_clock::now() <= endtime) {
            auto [makespan, agent_id, task_id] = assignment_queue.top();
            assignment_queue.pop();
            if (free_agents.count(agent_id) && free_tasks.count(task_id)) {
                task_assignments[task_id] = agent_id;
                proposed_schedule[agent_id] = task_id;
                free_agents.erase(agent_id);
                free_tasks.erase(task_id);
            }
        }
        for (int aid : free_agents) proposed_schedule[aid] = -1;
    } else {
        auto it = free_agents.begin();
        while (it != free_agents.end()) {
            if (std::chrono::steady_clock::now() > endtime) break;
            int i = *it;
            int min_task_i = -1, min_task_makespan = INT_MAX, count = 0;
            for (int t_id : free_tasks) {
                if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
                int dist = task_path_cost(env, env->curr_states.at(i).location, env->task_pool[t_id]) +
                           7 * scheduler_distance(env, env->curr_states.at(i).location,
                                                  env->task_pool[t_id].locations[0]);
                if (dist < min_task_makespan) { min_task_i = t_id; min_task_makespan = dist; }
                count++;
            }
            if (min_task_i != -1) {
                task_assignments[min_task_i] = i;
                proposed_schedule[i] = min_task_i;
                it = free_agents.erase(it);
                free_tasks.erase(min_task_i);
            } else { proposed_schedule[i] = -1; ++it; }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// initialize / plan（顶层入口，与原版 plan() 路由一致）
// ═══════════════════════════════════════════════════════════════════════════════

static bool s_initialized = false;

static void do_initialize(SharedEnvironment* env, int preprocess_time_limit)
{
    if (env->map_name[0] == 's') {
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        free_agents.clear();
        free_tasks.clear();
        DefaultPlanner::schedule_initialize(limit, env);
        s_initialized = true;
        return;
    }

    if (!env->map_name.empty() && env->map_name[0] == 'r') {
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        free_agents.clear();
        free_tasks.clear();
        clusters.clear();
        task_to_cluster.clear();
        last_clustered_free_tasks_count = 0;
        permanently_excluded_agents.clear();
        room_assign_initialized = false;
        rng.seed(42);
        DefaultPlanner::schedule_initialize(limit, env);
        s_initialized = true;
        return;
    }

    if (env->map_name[0] == 'o' || env->map_name[0] == 'w') {
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        free_agents.clear();
        free_tasks.clear();
        ow_initQuadTree(env);
        DefaultPlanner::schedule_initialize(limit, env);
        s_initialized = true;
        return;
    }

    // 默认分支（maze 等）：初始化 PrecomputedHeuristicTable
    free_agents.clear();
    free_tasks.clear();
    task_assignments.clear();
    location_congestion.clear();

    DefaultPlanner::schedule_initialize(preprocess_time_limit, env);
    pht_initialize(env);
    s_initialized = true;
}

static void do_plan(SharedEnvironment* env, int time_limit, std::vector<int>& proposed_schedule)
{
    if (env->map_name[0] == 's') {
        plan_s(env, time_limit, proposed_schedule);
        return;
    }

    if (env->num_of_agents > 300 && env->map_name[0] == 'r') {
        plan_large_r(env, time_limit, proposed_schedule);
        return;
    }

    if (env->map_name[0] == 'o' || env->map_name[0] == 'w') {
        plan_ow(env, time_limit, proposed_schedule);
        return;
    }

    if (env->num_of_agents < 300 && env->map_name[0] == 'r') {
        plan_small_r(env, time_limit, proposed_schedule);
        return;
    }

    schedule_plan(env, time_limit, proposed_schedule);
}

} // namespace dhfull_internal

// ═══════════════════════════════════════════════════════════════════════════════
// 对外接口
// ═══════════════════════════════════════════════════════════════════════════════

void DHFULLScheduler::initialize(SharedEnvironment* env, int preprocess_time_limit_ms)
{
    dhfull_internal::do_initialize(env, preprocess_time_limit_ms);
}

void DHFULLScheduler::schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env)
{
    dhfull_internal::do_plan(env, time_limit_ms, proposed_schedule);
}

} // namespace optsvr
