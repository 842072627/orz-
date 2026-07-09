// optsvr/schedule/OptMazeSchedule/OptMazeScheduler.cpp
// maze 1500tick 短时优化：PHT全对最短路 + Reclaim + DHFULLSchedule r图打分 + 匈牙利/贪心
// 打分沿用已验证的 r<300 公式: -(0.9*(dist_first*6 + path_dist) - 0.1*density_start + 0.2*density_end)

#include "OptMazeScheduler.h"
#include "SharedEnv.h"
#include "scheduler.h"
#include "const.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace orz_engine {
namespace optmaze_internal {

// ═══════════════════════════════════════════════════════════════════════════════
// PrecomputedHeuristicTable（maze 地图较小，全对最短路 O(1) 查表）
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int kThreadCount = 32;
static constexpr int kBlockedDistance = -1;
static constexpr char kCacheMagic[8] = {'P', 'H', 'E', 'U', 'R', 'V', '1', '\0'};
static constexpr char kDefaultCacheDir[] = "precomputed";
static constexpr size_t kMaxPrecomputeBytes = 20ull * 1024ull * 1024ull * 1024ull;
static constexpr int kMaxAgentsForPrecomputation = 9999;

struct CacheHeader {
    char magic[8];
    int rows, cols;
    long long free_cells;
};

static std::string pht_map_name;
static std::vector<int> pht_cell_id;        // location → free_cell_id (-1 if wall)
static std::vector<int> pht_location;       // free_cell_id → location
static std::vector<std::vector<int>> pht_neighbors;
static std::vector<int16_t> pht_distances;
static long long pht_n_free = 0;
static bool pht_ready = false;

static bool pht_get_size(size_t& entries, size_t& bytes) {
    unsigned __int128 n = pht_n_free;
    unsigned __int128 e = n * (n + 1) / 2;
    unsigned __int128 b = e * sizeof(int16_t);
    if (e > std::numeric_limits<size_t>::max() || b > std::numeric_limits<size_t>::max()) return false;
    entries = static_cast<size_t>(e);
    bytes = static_cast<size_t>(b);
    return true;
}

static size_t pht_index(long long a, long long b) {
    if (a > b) std::swap(a, b);
    return static_cast<size_t>(a * pht_n_free - a * (a - 1) / 2 + b - a);
}

static std::string pht_cache_path(const SharedEnvironment* env) {
    if (env->file_storage_path.empty()) return "";
    return env->file_storage_path + "/" + env->map_name + "_optmaze_get_h_v1.bin";
}

static bool pht_read_cache(const SharedEnvironment* env) {
    std::string path = pht_cache_path(env);
    if (path.empty()) return false;
    std::ifstream in(path, std::ios::binary);
    if (!in.good()) return false;
    CacheHeader h{};
    in.read(reinterpret_cast<char*>(&h), sizeof(h));
    if (in.fail() || std::memcmp(h.magic, kCacheMagic, 8) != 0 ||
        h.rows != env->rows || h.cols != env->cols || h.free_cells != pht_n_free)
        return false;
    size_t entries, bytes;
    if (!pht_get_size(entries, bytes) || bytes > kMaxPrecomputeBytes) return false;
    pht_distances.resize(entries);
    in.read(reinterpret_cast<char*>(pht_distances.data()), entries * sizeof(int16_t));
    if (in.fail()) { pht_distances.clear(); return false; }
    std::cerr << "[OptMaze] PHT cache loaded: " << path << "\n";
    return true;
}

static void pht_write_cache(const SharedEnvironment* env) {
    std::string path = pht_cache_path(env);
    if (path.empty()) return;
    std::ofstream out(path, std::ios::binary);
    if (!out.good()) return;
    CacheHeader h{};
    std::memcpy(h.magic, kCacheMagic, 8);
    h.rows = env->rows; h.cols = env->cols; h.free_cells = pht_n_free;
    out.write(reinterpret_cast<const char*>(&h), sizeof(h));
    out.write(reinterpret_cast<const char*>(pht_distances.data()), pht_distances.size() * sizeof(int16_t));
    std::cerr << "[OptMaze] PHT cache saved: " << path << "\n";
}

static void pht_compute() {
    size_t entries, bytes;
    (void)bytes;
    if (!pht_get_size(entries, bytes)) throw std::runtime_error("PHT overflow");
    pht_distances.assign(entries, static_cast<int16_t>(kBlockedDistance));

#ifdef _OPENMP
#pragma omp parallel num_threads(kThreadCount)
    {
        std::vector<int> dist(pht_n_free, kBlockedDistance);
        std::queue<int> q;
#pragma omp for schedule(dynamic, 1)
        for (long long src = 0; src < pht_n_free; ++src) {
            std::fill(dist.begin(), dist.end(), kBlockedDistance);
            while (!q.empty()) q.pop();
            dist[src] = 0; q.push(src);
            while (!q.empty()) {
                int cur = q.front(); q.pop();
                int d = dist[cur];
                if (cur >= src) pht_distances[pht_index(src, cur)] = static_cast<int16_t>(d);
                for (int nb : pht_neighbors[cur]) {
                    if (dist[nb] != kBlockedDistance) continue;
                    dist[nb] = d + 1; q.push(nb);
                }
            }
        }
    }
#else
    std::vector<int> dist(pht_n_free, kBlockedDistance);
    std::queue<int> q;
    for (long long src = 0; src < pht_n_free; ++src) {
        std::fill(dist.begin(), dist.end(), kBlockedDistance);
        while (!q.empty()) q.pop();
        dist[src] = 0; q.push(src);
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int d = dist[cur];
            if (cur >= src) pht_distances[pht_index(src, cur)] = static_cast<int16_t>(d);
            for (int nb : pht_neighbors[cur]) {
                if (dist[nb] != kBlockedDistance) continue;
                dist[nb] = d + 1; q.push(nb);
            }
        }
    }
#endif
}

static void pht_initialize(SharedEnvironment* env) {
    if (pht_ready && pht_map_name == env->map_name) return;
    pht_distances.clear(); pht_distances.shrink_to_fit();
    pht_ready = false;

    if (env->file_storage_path.empty()) env->file_storage_path = kDefaultCacheDir;
    std::error_code ec;
    std::filesystem::create_directories(env->file_storage_path, ec);

    // 构建图
    pht_cell_id.assign(env->map.size(), -1);
    pht_location.clear();
    for (int loc = 0; loc < static_cast<int>(env->map.size()); ++loc) {
        if (env->map[loc] != 0) continue;
        pht_cell_id[loc] = static_cast<int>(pht_location.size());
        pht_location.push_back(loc);
    }
    pht_n_free = pht_location.size();
    pht_neighbors.assign(pht_n_free, {});
    for (long long id = 0; id < pht_n_free; ++id) {
        int loc = pht_location[id], row = loc / env->cols, col = loc % env->cols;
        int cands[4] = {row > 0 ? loc - env->cols : -1, row + 1 < env->rows ? loc + env->cols : -1,
                        col > 0 ? loc - 1 : -1, col + 1 < env->cols ? loc + 1 : -1};
        for (int nl : cands) {
            if (nl >= 0 && pht_cell_id[nl] != -1) pht_neighbors[id].push_back(pht_cell_id[nl]);
        }
    }

    if (env->num_of_agents > kMaxAgentsForPrecomputation) {
        std::cerr << "[OptMaze] skip PHT: agents=" << env->num_of_agents << "\n";
        pht_map_name = env->map_name; pht_ready = true; return;
    }
    size_t entries, bytes;
    if (!pht_get_size(entries, bytes) || bytes > kMaxPrecomputeBytes) {
        std::cerr << "[OptMaze] skip PHT: too large\n";
        pht_map_name = env->map_name; pht_ready = true; return;
    }

    if (!pht_read_cache(env)) {
        std::cerr << "[OptMaze] computing PHT (" << kThreadCount << " threads)...\n";
        pht_compute();
        pht_write_cache(env);
    }
    pht_map_name = env->map_name; pht_ready = true;
}

static bool pht_available() { return pht_ready && !pht_distances.empty(); }

static int pht_get(int src, int dst) {
    if (src == dst) return 0;
    if (src < 0 || dst < 0 || src >= (int)pht_cell_id.size() || dst >= (int)pht_cell_id.size()) return INT_MAX;
    int a = pht_cell_id[src], b = pht_cell_id[dst];
    if (a == -1 || b == -1) return INT_MAX;
    return pht_distances[pht_index(a, b)];
}

// ═══════════════════════════════════════════════════════════════════════════════
// 距离查询
// ═══════════════════════════════════════════════════════════════════════════════

static int get_dist(SharedEnvironment* env, int src, int dst) {
    if (pht_available()) return pht_get(src, dst);
    return DefaultPlanner::get_h(env, src, dst);
}

static int task_path_cost(SharedEnvironment* env, int agent_loc, const Task& task) {
    if (task.locations.empty()) return 0;
    int total = get_dist(env, agent_loc, task.locations[0]);
    int cur = task.locations[0];
    for (size_t i = 1; i < task.locations.size(); ++i) {
        total += get_dist(env, cur, task.locations[i]);
        cur = task.locations[i];
    }
    return total;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 运行时可调参数（initialize 时从环境变量加载）
// ═══════════════════════════════════════════════════════════════════════════════

static int    p_cell_size         = 2;
static double p_dist_first_weight = 10.0;   // R2冠军：首点距离权重10
static double p_dist_overall      = 0.9;
static double p_density_start_w   = 0.0;    // R2冠军：关闭密度
static double p_density_end_w     = 0.0;    // R2冠军：关闭密度
static int    p_reclaim_margin    = 1;
static bool   p_time_filter       = false;
static int    p_total_ticks       = 1500;
static int    p_greedy_mode       = 0;
static int    p_lookahead         = 1;      // R2冠军：启用预瞻
static double p_lookahead_w       = 0.3;    // R2冠军：预瞻权重0.3
static double p_congestion_w      = 0.0;
static int    p_dynamic_fw        = 0;
static double p_fw_early          = 6.0;
static double p_fw_late           = 15.0;
static int    p_lookahead_topk    = 1;

static void load_params() {
    auto getd = [](const char* name, double def) -> double {
        const char* v = std::getenv(name);
        return v ? std::atof(v) : def;
    };
    auto geti = [](const char* name, int def) -> int {
        const char* v = std::getenv(name);
        return v ? std::atoi(v) : def;
    };
    p_cell_size         = geti("OPT_MAZE_CELL_SIZE", 2);
    p_dist_first_weight = getd("OPT_MAZE_DIST_FIRST_W", 10.0);
    p_dist_overall      = getd("OPT_MAZE_DIST_OVERALL", 0.9);
    p_density_start_w   = getd("OPT_MAZE_DENSITY_START", 0.0);
    p_density_end_w     = getd("OPT_MAZE_DENSITY_END", 0.0);
    p_reclaim_margin    = geti("OPT_MAZE_RECLAIM_MARGIN", 1);
    p_time_filter       = geti("OPT_MAZE_TIME_FILTER", 0) != 0;
    p_total_ticks       = geti("OPT_MAZE_TOTAL_TICKS", 1500);
    p_greedy_mode       = geti("OPT_MAZE_GREEDY", 0);
    p_lookahead         = geti("OPT_MAZE_LOOKAHEAD", 1);
    p_lookahead_w       = getd("OPT_MAZE_LOOKAHEAD_W", 0.3);
    p_congestion_w      = getd("OPT_MAZE_CONGESTION_W", 0.0);
    p_dynamic_fw        = geti("OPT_MAZE_DYNAMIC_FW", 0);
    p_fw_early          = getd("OPT_MAZE_FW_EARLY", 6.0);
    p_fw_late           = getd("OPT_MAZE_FW_LATE", 15.0);
    p_lookahead_topk    = geti("OPT_MAZE_LOOKAHEAD_TOPK", 1);
    std::cerr << "[OptMaze] params: first_w=" << p_dist_first_weight
              << " overall=" << p_dist_overall
              << " d_start=" << p_density_start_w
              << " d_end=" << p_density_end_w
              << " cell=" << p_cell_size
              << " reclaim_m=" << p_reclaim_margin
              << " time_filter=" << p_time_filter
              << " greedy=" << p_greedy_mode
              << " lookahead=" << p_lookahead
              << " la_w=" << p_lookahead_w
              << " la_topk=" << p_lookahead_topk
              << " congest_w=" << p_congestion_w
              << " dyn_fw=" << p_dynamic_fw
              << " fw_early=" << p_fw_early
              << " fw_late=" << p_fw_late
              << " ticks=" << p_total_ticks << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// 全局状态
// ═══════════════════════════════════════════════════════════════════════════════

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, int> task_assignments;

// ═══════════════════════════════════════════════════════════════════════════════
// Reclaim：回收来不及完成的已分配任务
// ═══════════════════════════════════════════════════════════════════════════════

static bool can_reclaim(SharedEnvironment* env, int agent_id, const Task& task) {
    if (task.idx_next_loc != 0 || task.locations.empty() || task.agent_assigned != agent_id) return false;
    if (agent_id < 0 || agent_id >= (int)env->curr_task_schedule.size() ||
        env->curr_task_schedule[agent_id] != task.task_id) return false;
    const State& st = env->system_states.at(agent_id);
    int opt_loc = st.location;
    int remaining = 0;
    if (st.counter.count > 0 && st.current_action != Action::NA) {
        remaining = std::max(0, st.counter.maxCount - st.counter.count);
        if (st.current_action == Action::FW) {
            const int mv[4] = {1, env->cols, -1, -env->cols};
            opt_loc += mv[st.orientation];
        }
    }
    int dist = get_dist(env, opt_loc, task.locations[0]);
    int ticks = remaining + dist * env->max_counter;
    int action_time = std::max(1, env->action_time);
    int horizon = (env->min_planner_communication_time + action_time - 1) / action_time + p_reclaim_margin;
    return ticks > horizon;
}

static void rebuild_state(SharedEnvironment* env, std::vector<int>& sched) {
    free_agents.clear(); free_tasks.clear(); task_assignments.clear();
    for (int a = 0; a < (int)env->curr_task_schedule.size(); ++a) {
        if (env->curr_task_schedule[a] == -1) { free_agents.insert(a); sched[a] = -1; }
    }
    for (const auto& [tid, task] : env->task_pool) {
        if (task.agent_assigned == -1) free_tasks.insert(tid);
        else task_assignments[tid] = task.agent_assigned;
    }
}

static void reclaim_tasks(SharedEnvironment* env, std::vector<int>& sched) {
    for (auto it = task_assignments.begin(); it != task_assignments.end(); ) {
        int tid = it->first, aid = it->second;
        auto ti = env->task_pool.find(tid);
        if (ti == env->task_pool.end()) { it = task_assignments.erase(it); continue; }
        if (can_reclaim(env, aid, ti->second)) {
            free_tasks.insert(tid); free_agents.insert(aid); sched[aid] = -1;
            it = task_assignments.erase(it);
        } else { ++it; }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 打分函数
// ═══════════════════════════════════════════════════════════════════════════════

// 拥堵统计：统计 agent 当前位置与任务首点之间区域内的其他 agent 数
static int count_congestion(SharedEnvironment* env, int agent_loc, int target_loc) {
    int ar = agent_loc / env->cols, ac = agent_loc % env->cols;
    int tr = target_loc / env->cols, tc = target_loc % env->cols;
    int rmin = std::min(ar, tr), rmax = std::max(ar, tr);
    int cmin = std::min(ac, tc), cmax = std::max(ac, tc);
    // 扩展1格边界
    rmin = std::max(0, rmin - 1); rmax = std::min(env->rows - 1, rmax + 1);
    cmin = std::max(0, cmin - 1); cmax = std::min(env->cols - 1, cmax + 1);
    int cnt = 0;
    for (int a = 0; a < env->num_of_agents; ++a) {
        int loc = env->curr_states.at(a).location;
        int r = loc / env->cols, c = loc % env->cols;
        if (r >= rmin && r <= rmax && c >= cmin && c <= cmax) cnt++;
    }
    return cnt - 1;  // 减去自己
}

// 预瞻：完成任务后，离最近未分配任务的距离（支持 top-K 平均）
static double lookahead_bonus(SharedEnvironment* env, const Task& task) {
    int end_loc = task.locations.back();
    int k = p_lookahead_topk;
    if (k <= 1) {
        // top-1: 最近的
        int min_dist = INT_MAX;
        for (const auto& [tid, other] : env->task_pool) {
            if (tid == task.task_id || other.locations.empty() || other.agent_assigned != -1) continue;
            int d = get_dist(env, end_loc, other.locations[0]);
            if (d < min_dist) min_dist = d;
        }
        return (min_dist == INT_MAX) ? 0.0 : static_cast<double>(min_dist);
    } else {
        // top-K: 最近K个的平均
        std::vector<int> dists;
        for (const auto& [tid, other] : env->task_pool) {
            if (tid == task.task_id || other.locations.empty() || other.agent_assigned != -1) continue;
            dists.push_back(get_dist(env, end_loc, other.locations[0]));
        }
        if (dists.empty()) return 0.0;
        std::sort(dists.begin(), dists.end());
        int cnt = std::min(k, (int)dists.size());
        double sum = 0;
        for (int i = 0; i < cnt; ++i) sum += dists[i];
        return sum / cnt;
    }
}

static double compute_cost(SharedEnvironment* env, int agent_id, int task_id) {
    auto ti = env->task_pool.find(task_id);
    if (ti == env->task_pool.end()) return 1e9;
    const Task& task = ti->second;
    if (task.locations.empty()) return 1e9;

    int agent_loc = env->curr_states.at(agent_id).location;
    int dist_first = get_dist(env, agent_loc, task.locations[0]);

    // 时间过滤：如果剩余 tick 不够完成该任务，跳过
    if (p_time_filter) {
        int full_cost = task_path_cost(env, agent_loc, task);
        int ticks_needed = full_cost * env->max_counter;
        int remaining_ticks = p_total_ticks - env->curr_timestep;
        if (ticks_needed > remaining_ticks) return 1e9;
    }

    // 动态 first_w：前半程用 fw_early，后半程用 fw_late
    double eff_first_w = p_dist_first_weight;
    if (p_dynamic_fw) {
        double progress = static_cast<double>(env->curr_timestep) / p_total_ticks;
        eff_first_w = (progress < 0.5) ? p_fw_early : p_fw_late;
    }

    double total_dist = dist_first * eff_first_w;

    // 全程路径距离
    int cur = agent_loc;
    for (int loc : task.locations) { total_dist += get_dist(env, cur, loc); cur = loc; }

    double cost = p_dist_overall * total_dist;

    // 密度计算
    if (p_density_start_w > 0 || p_density_end_w > 0) {
        int start_loc = task.locations[0];
        int end_loc = task.locations.back();
        int cs = p_cell_size;
        int gx = (start_loc / env->cols) / cs, gy = (start_loc % env->cols) / cs;
        int egx = (end_loc / env->cols) / cs, egy = (end_loc % env->cols) / cs;
        int density_start = 0, density_end = 0;
        for (const auto& [oid, other] : env->task_pool) {
            if (oid == task_id || other.locations.empty()) continue;
            int os = other.locations[0];
            int ox = (os / env->cols) / cs, oy = (os % env->cols) / cs;
            if (std::abs(ox - gx) <= 1 && std::abs(oy - gy) <= 1) density_start++;
            if (std::abs(ox - egx) <= 1 && std::abs(oy - egy) <= 1) density_end++;
        }
        cost += -p_density_start_w * density_start + p_density_end_w * density_end;
    }

    // 拥堵惩罚
    if (p_congestion_w > 0) {
        int cong = count_congestion(env, agent_loc, task.locations[0]);
        cost += p_congestion_w * cong;
    }

    // 预瞻：完成后离下一个最近任务的距离（越近越好）
    if (p_lookahead) {
        double la = lookahead_bonus(env, task);
        cost += p_lookahead_w * la;
    }

    return cost;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 匈牙利算法（double 版）
// ═══════════════════════════════════════════════════════════════════════════════

static void hungarian(std::vector<std::vector<double>>& C, std::vector<int>& assign) {
    int n = C.size(); if (n == 0) return;
    int m = C[0].size(); if (m == 0) return;
    bool trans = n > m;
    if (trans) {
        std::swap(n, m);
        std::vector<std::vector<double>> T(m, std::vector<double>(n));
        for (int i = 0; i < m; ++i) for (int j = 0; j < n; ++j) T[i][j] = C[j][i];
        C.swap(T);
    }
    std::vector<double> u(n+1, 0), v(m+1, 0);
    std::vector<int> p(m+1, 0), way(m+1, 0);
    for (int i = 1; i <= n; ++i) {
        p[0] = i; int j0 = 0;
        std::vector<double> minv(m+1, 1e18);
        std::vector<bool> used(m+1, false);
        do {
            used[j0] = true; int i0 = p[j0]; double delta = 1e18; int j1 = 0;
            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                double cur = C[i0-1][j-1] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else minv[j] -= delta;
            }
            j0 = j1;
        } while (p[j0] != 0);
        do { int j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
    }
    assign.assign(n, -1);
    for (int j = 1; j <= m; ++j) if (p[j]) assign[p[j]-1] = j-1;
    if (trans) {
        std::vector<int> tmp(m, -1);
        for (int i = 0; i < n; ++i) if (assign[i] != -1) tmp[assign[i]] = i;
        assign.swap(tmp);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 核心调度
// ═══════════════════════════════════════════════════════════════════════════════

static void plan(SharedEnvironment* env, int time_limit, std::vector<int>& sched) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    auto endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    rebuild_state(env, sched);
    reclaim_tasks(env, sched);

    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::vector<int> tasks(free_tasks.begin(), free_tasks.end());
    int na = agents.size(), nt = tasks.size();
    if (na == 0 || nt == 0) { for (int a : free_agents) sched[a] = -1; return; }

    // 贪心模式：按 agent 排序，每个 agent 选最优任务
    if (p_greedy_mode) {
        // 按 agent 到最近任务的距离排序（距离最短的 agent 优先选）
        std::sort(agents.begin(), agents.end(), [&](int a, int b) {
            int loc_a = env->curr_states.at(a).location;
            int loc_b = env->curr_states.at(b).location;
            int min_a = INT_MAX, min_b = INT_MAX;
            for (int tid : tasks) {
                auto it = env->task_pool.find(tid);
                if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                min_a = std::min(min_a, get_dist(env, loc_a, it->second.locations[0]));
                min_b = std::min(min_b, get_dist(env, loc_b, it->second.locations[0]));
            }
            return min_a < min_b;
        });
        std::unordered_set<int> used_tasks;
        for (int aid : agents) {
            if (std::chrono::steady_clock::now() > endtime) break;
            double best_cost = 1e9;
            int best_tid = -1;
            for (int tid : tasks) {
                if (used_tasks.count(tid)) continue;
                double c = compute_cost(env, aid, tid);
                if (c < best_cost) { best_cost = c; best_tid = tid; }
            }
            if (best_tid >= 0 && best_cost < 1e8) {
                sched[aid] = best_tid;
                used_tasks.insert(best_tid);
                free_agents.erase(aid); free_tasks.erase(best_tid);
            }
        }
    }
    // 匈牙利模式（小规模 < 300 agents）
    else if (na < 300) {
        std::vector<std::vector<double>> cost(na, std::vector<double>(nt, 0.0));
        for (int i = 0; i < na; ++i) {
            if (std::chrono::steady_clock::now() > endtime) break;
            for (int j = 0; j < nt; ++j) {
                if (j % 50 == 0 && std::chrono::steady_clock::now() > endtime) break;
                cost[i][j] = compute_cost(env, agents[i], tasks[j]);
            }
        }
        std::vector<int> assign;
        hungarian(cost, assign);
        for (int i = 0; i < na; ++i) {
            if (assign[i] >= 0 && assign[i] < nt && cost[i][assign[i]] < 1e8) {
                sched[agents[i]] = tasks[assign[i]];
                free_agents.erase(agents[i]);
                free_tasks.erase(tasks[assign[i]]);
            }
        }
    } else {
        // 大规模：优先队列贪心
        using Entry = std::tuple<double, int, int>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;
        for (int i = 0; i < na; ++i) {
            if (std::chrono::steady_clock::now() > endtime) break;
            for (int j = 0; j < nt; ++j) {
                if (j % 20 == 0 && std::chrono::steady_clock::now() > endtime) break;
                double c = compute_cost(env, agents[i], tasks[j]);
                if (c < 1e8) pq.emplace(c, agents[i], tasks[j]);
            }
        }
        while (!pq.empty() && std::chrono::steady_clock::now() <= endtime) {
            auto [c, aid, tid] = pq.top(); pq.pop();
            if (free_agents.count(aid) && free_tasks.count(tid)) {
                sched[aid] = tid;
                free_agents.erase(aid); free_tasks.erase(tid);
            }
        }
    }

    for (int a : free_agents) sched[a] = -1;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 初始化 & 入口
// ═══════════════════════════════════════════════════════════════════════════════

static void do_init(SharedEnvironment* env, int preprocess_time_limit) {
    free_agents.clear(); free_tasks.clear(); task_assignments.clear();
    load_params();
    DefaultPlanner::schedule_initialize(preprocess_time_limit, env);
    pht_initialize(env);
    std::cerr << "[OptMaze] initialized (maze-1500tick)\n";
}

static void do_plan(SharedEnvironment* env, int time_limit, std::vector<int>& sched) {
    plan(env, time_limit, sched);
}

} // namespace optmaze_internal

// ═══════════════════════════════════════════════════════════════════════════════
// 对外接口
// ═══════════════════════════════════════════════════════════════════════════════

void OptMazeScheduler::initialize(SharedEnvironment* env, int preprocess_time_limit_ms) {
    optmaze_internal::do_init(env, preprocess_time_limit_ms);
}

void OptMazeScheduler::schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    optmaze_internal::do_plan(env, time_limit_ms, proposed_schedule);
}

} // namespace orz_engine
