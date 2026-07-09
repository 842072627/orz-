// optsvr/schedule/OptRoomSchedule/OptRoomScheduler.cpp
// room 3000tick 优化：PHT全对最短路 + 房间感知 + 门级潮汐 + 密度控制 + Reassign + 限流
// 第6轮优化：引入潮汐车道、房间密度上限、任务密度均衡、Reassign、限流比例

#include "OptRoomScheduler.h"
#include "SharedEnv.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "ScheduleStatsExport.h"

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace orz_engine {
namespace optroom_internal {

// ═══════════════════════════════════════════════════════════════════════════════
// PrecomputedHeuristicTable（room 地图 64×64 约 3646 free cells，全对最短路）
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
static std::vector<int> pht_cell_id;
static std::vector<int> pht_location;
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
    return env->file_storage_path + "/" + env->map_name + "_optroom_get_h_v1.bin";
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
    std::cerr << "[OptRoom] PHT cache loaded: " << path << "\n";
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
    std::cerr << "[OptRoom] PHT cache saved: " << path << "\n";
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
        std::cerr << "[OptRoom] skip PHT: agents=" << env->num_of_agents << "\n";
        pht_map_name = env->map_name; pht_ready = true; return;
    }
    size_t entries, bytes;
    if (!pht_get_size(entries, bytes) || bytes > kMaxPrecomputeBytes) {
        std::cerr << "[OptRoom] skip PHT: too large\n";
        pht_map_name = env->map_name; pht_ready = true; return;
    }

    if (!pht_read_cache(env)) {
        std::cerr << "[OptRoom] computing PHT (" << kThreadCount << " threads)...\n";
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
// Distance query
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
// Room-Aware Topology (same as before)
// ═══════════════════════════════════════════════════════════════════════════════

static constexpr int kNumRooms = 16;
static constexpr int kWallLines[] = {0, 16, 32, 48, 63};

static std::vector<int> room_of_cell;
static std::vector<std::vector<int>> room_cells;
static std::vector<int> room_gate_count;
static std::vector<bool> is_bottleneck_room;
static bool topo_ready = false;

struct Gate {
    int location;
    int room_a, room_b;
};
static std::vector<Gate> gates;
static std::unordered_set<int> gate_locations;

// Room distribution
static std::vector<int> agents_in_room;

static bool is_wall_line(int v, const int lines[], int n) {
    for (int i = 0; i < n; ++i) if (lines[i] == v) return true;
    return false;
}

static int compute_room_id(int row, int col) {
    if (is_wall_line(row, kWallLines, 5) || is_wall_line(col, kWallLines, 5)) return -1;
    int rr = (row < 16) ? 0 : (row < 32) ? 1 : (row < 48) ? 2 : 3;
    int cc = (col < 16) ? 0 : (col < 32) ? 1 : (col < 48) ? 2 : 3;
    return rr * 4 + cc;
}

static void topo_initialize(SharedEnvironment* env) {
    if (topo_ready) return;
    int rows = env->rows, cols = env->cols;
    
    room_of_cell.assign(rows * cols, -1);
    room_cells.assign(kNumRooms, {});
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int loc = r * cols + c;
            if (env->map[loc] != 0) continue;
            int rid = compute_room_id(r, c);
            room_of_cell[loc] = rid;
            if (rid >= 0) room_cells[rid].push_back(loc);
        }
    }
    
    gates.clear(); gate_locations.clear();
    room_gate_count.assign(kNumRooms, 0);
    // Horizontal wall lines
    for (int rwall : {16, 32, 48}) {
        for (int c = 1; c < cols - 1; ++c) {
            int loc = rwall * cols + c;
            if (env->map[loc] != 0) continue;
            int ra = compute_room_id(rwall - 1, c);
            int rb = compute_room_id(rwall + 1, c);
            if (ra >= 0 && rb >= 0) {
                gates.push_back({loc, ra, rb});
                gate_locations.insert(loc);
                room_gate_count[ra]++;
                room_gate_count[rb]++;
            }
        }
    }
    // Vertical wall lines
    for (int cwall : {16, 32, 48}) {
        for (int r = 1; r < rows - 1; ++r) {
            int loc = r * cols + cwall;
            if (env->map[loc] != 0) continue;
            int ra = compute_room_id(r, cwall - 1);
            int rb = compute_room_id(r, cwall + 1);
            if (ra >= 0 && rb >= 0) {
                gates.push_back({loc, ra, rb});
                gate_locations.insert(loc);
                room_gate_count[ra]++;
                room_gate_count[rb]++;
            }
        }
    }
    
    is_bottleneck_room.assign(kNumRooms, false);
    for (int i = 0; i < kNumRooms; ++i) {
        if (room_gate_count[i] <= 2) is_bottleneck_room[i] = true;
    }
    
    agents_in_room.assign(kNumRooms, 0);
    
    std::cerr << "[OptRoom] topology: " << gates.size() << " gates, bottleneck rooms:";
    for (int i = 0; i < kNumRooms; ++i) {
        if (is_bottleneck_room[i]) std::cerr << " R" << i << "(g=" << room_gate_count[i] << ")";
    }
    std::cerr << "\n";
    topo_ready = true;
}

static int get_room(int loc) {
    if (loc < 0 || loc >= (int)room_of_cell.size()) return -1;
    return room_of_cell[loc];
}

// ═══════════════════════════════════════════════════════════════════════════════
// Runtime parameters
// ═══════════════════════════════════════════════════════════════════════════════

// Core scoring
static double p_dist_first_weight = 3.5;
static double p_dist_overall      = 0.9;
static double p_short_task_w      = 1.0;
static int    p_total_ticks       = 3000;
static int    p_reclaim_margin    = 1;

// Assign ratio (flow limiting)
static double p_assign_ratio      = 1.0;     // fraction of free agents to assign (0.0~1.0)
static int    p_match_mode        = 0;       // 0=truncate+hungarian, 1=greedy(cap), 2=full_hungarian+topK, 3=mincostflow stable-snatch
static int    p_snatch_min_pickup_distance = 10;
static double p_snatch_min_abs_improve     = 6.0;
static double p_snatch_min_rel_improve     = 0.10;

// Reassign
static int    p_reassign_enable   = 0;       // 1=enable reassign of unstarted tasks
static int    p_reassign_min_dist = 5;       // distance protection for reassign

// Time safety filter
static double p_time_safety       = 1.0;     // multiplier on estimated cost for time filter (>1 = stricter)

// Post-Hungarian swap (保留，默认关闭)
static int    p_post_swap         = 0;       // 1=enable post-hungarian swap for worst pairs

// Chain bonus (保留，默认关闭)
static int    p_chain_bonus       = 0;       // 1=enable chain bonus
static double p_chain_bonus_w     = 3.0;     // cost reduction for same-room next task

// Distance power (非线性距离打分)
static double p_dist_pow          = 1.0;     // 1.0=linear, 0.5=sqrt, 0.7=sublinear

// Greedy TopK (剪枝加速)
static int    p_greedy_topk       = 0;       // 0=no pruning, >0=per-agent top K tasks

// ---- R14 新策略参数 ----
// 多轮贪心: 第一轮分配后，取消 bottom-K 最差配对，第二轮重新匹配
static int    p_greedy_rounds     = 1;       // 1=单轮(默认), 2=两轮优化
static double p_greedy_drop_pct   = 0.2;     // 每轮末尾丢弃最差配对比例

// 批量延迟分配: 每N tick才做一次分配(积累更多task做全局最优)
static int    p_batch_delay       = 3;       // R15验证最优: 每3tick批量分配

// ---- R16 新策略参数 ----
// Legacy greedy-only extras. match_mode=3 uses compute_room0628_cost() and ignores these.
static int    p_priority_decay    = 0;
static int    p_priority_decay_start_ticks = 3;
static double p_priority_decay_w  = 0.5;

// 局部交换: greedy分配后，对最差配对尝试2-opt局部搜索
static int    p_local_swap        = 1;       // R16验证有效: 启用
static int    p_local_swap_k      = 10;      // 检查最差10对配对





// ---- R18 新策略参数 ----
// 动态batch: 根据free_tasks数量动态调整batch延迟
static int    p_dynamic_batch     = 1;       // R18验证有效: 启用
static int    p_dynamic_batch_low = 1;       // tasks多时的batch(快速分配)
static int    p_dynamic_batch_high= 5;       // tasks少时的batch(积累优化)
static int    p_dynamic_batch_pivot= 5;      // free_tasks数量分界点



static int    p_task_gravity      = 0;
static double p_task_gravity_w    = 1.0;
static int    p_task_gravity_r    = 20;

// ═══════════════════════════════════════════════════════════════════════════════
// 以下策略经测试确认无效/有害，保留变量定义供编译通过，永久关闭(=0)
// ═══════════════════════════════════════════════════════════════════════════════
static int    p_gate_cong         = 0;
static double p_gate_cong_w       = 2.0;
static std::vector<int> gate_usage_count;
static int    p_phase_batch       = 0;
static double p_phase_batch_split = 0.3;
static int    p_multi_pick        = 0;
static double p_multi_pick_thresh = 3.0;
static int    p_neighbor_coord    = 0;
static double p_neighbor_coord_w  = 2.0;
static int    p_completion_ratio  = 0;
static double p_completion_target = 200.0;
static int    p_dist_decay_fw     = 0;
static double p_dist_decay_near   = 1.0;
static double p_dist_decay_far    = 3.0;
static int    p_dist_decay_thresh = 10;
static int    p_age_sort          = 0;
static int    p_lookahead_chain   = 0;
static double p_lookahead_chain_w = 2.0;
static int    p_lookahead_chain_r = 15;
static int    p_utility_gate      = 0;
static double p_utility_gate_mult = 1.5;
static int    p_agent_shuffle     = 0;
static int    p_surplus_boost     = 0;
static double p_surplus_boost_thresh = 2.0;
static double p_surplus_boost_ratio  = 0.1;
static int    p_ratio_phase       = 0;
static double p_ratio_phase_early = 0.6;
static double p_ratio_phase_late  = 0.45;
static double p_ratio_phase_split = 0.4;
static int    p_cost_momentum     = 0;
static double p_cost_momentum_w   = 1.5;
static int    p_cost_momentum_r   = 15;
static int    p_idle_penalty      = 0;
static int    p_idle_penalty_base = 5;
static double p_idle_penalty_w    = 0.5;
static int    p_bipartite_refine  = 0;
static int    p_bipartite_refine_k= 6;
static int    p_confidence_gate   = 0;
static double p_confidence_thresh = 3.0;
static int    p_decay_annealing   = 0;
static double p_decay_annealing_max = 1.5;
static int    p_soft_batch        = 0;
static int    p_soft_batch_min    = 3;
static int    p_soft_batch_max_wait = 5;
static int    p_adaptive_conf     = 0;
static double p_adaptive_conf_low = 2.0;
static double p_adaptive_conf_high= 5.0;
static int    p_adaptive_conf_pivot = 5;
static int    p_weighted_shuffle  = 0;
static double p_weighted_shuffle_alpha = 3.0;
static int    p_dual_queue        = 0;
static int    p_dual_queue_thresh = 10;
static int    p_horizon_filter    = 0;
static int    p_horizon_filter_time = 2000;
static int    p_horizon_filter_min  = 3;
static int    p_recency_bonus     = 0;
static double p_recency_bonus_w   = 2.0;
static int    p_recency_bonus_window = 5;
static double p_cost_jitter       = 0.0;
static int    p_fw_phase          = 0;
static double p_fw_early          = 3.0;
static double p_fw_late           = 1.0;
static double p_task_wait_w       = 0.0;
static int    p_task_wait_base    = 10;
static double p_scarcity_ratio    = 0.0;
static int    p_regret_mode       = 0;
static double p_regret_thresh     = 2.0;
static int    p_adaptive_fw       = 0;
static double p_adaptive_fw_high  = 3.0;
static double p_adaptive_fw_low   = 1.0;
static double p_adaptive_fw_pivot = 1.0;
static int    p_cost_normalize    = 0;
static double p_normalize_base    = 30.0;

// Track last completed task's room per agent (for chain bonus)
static std::vector<int> agent_last_room;
// Track last completed task end location per agent (for momentum)
static std::vector<int> agent_last_end_loc;
// Track per-agent idle ticks (consecutive ticks without task)
static std::vector<int> agent_idle_ticks;

static void load_params() {
    auto& c = orz_engine::cfg();
    
    // Core
    p_dist_first_weight = c.opt_room_dist_first_w;
    p_dist_overall      = c.opt_room_dist_overall;
    p_short_task_w      = c.opt_room_short_task_w;
    p_total_ticks       = c.opt_room_total_ticks;
    p_reclaim_margin    = c.opt_room_reclaim_margin;
    
    // Assign ratio & matching
    p_assign_ratio      = c.opt_room_assign_ratio;
    p_match_mode        = c.opt_room_match_mode;
    p_snatch_min_pickup_distance = c.opt_room_snatch_min_pickup_distance;
    p_snatch_min_abs_improve     = c.opt_room_snatch_min_abs_improve;
    p_snatch_min_rel_improve     = c.opt_room_snatch_min_rel_improve;
    
    // Reassign
    p_reassign_enable   = c.opt_room_reassign;
    p_reassign_min_dist = c.opt_room_reassign_min_dist;

    // Time safety filter
    p_time_safety       = c.opt_room_time_safety;

    // Legacy greedy-mode knobs. They do not affect match_mode=3 stable-snatch.
    if (auto v = std::getenv("OPT_ROOM_GREEDY_ROUNDS"))  p_greedy_rounds  = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_GREEDY_DROP_PCT"))p_greedy_drop_pct= std::atof(v);
    if (auto v = std::getenv("OPT_ROOM_BATCH_DELAY"))     p_batch_delay     = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_LOCAL_SWAP"))      p_local_swap      = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_LOCAL_SWAP_K"))    p_local_swap_k    = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_DYNAMIC_BATCH"))       p_dynamic_batch       = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_DYNAMIC_BATCH_LOW"))   p_dynamic_batch_low   = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_DYNAMIC_BATCH_HIGH"))  p_dynamic_batch_high  = std::atoi(v);
    if (auto v = std::getenv("OPT_ROOM_DYNAMIC_BATCH_PIVOT")) p_dynamic_batch_pivot = std::atoi(v);

    std::cerr << "[OptRoom] params: fw=" << p_dist_first_weight
              << " short=" << p_short_task_w
              << " ratio=" << p_assign_ratio
              << " match=" << p_match_mode
              << " snatch_dist=" << p_snatch_min_pickup_distance
              << " snatch_abs=" << p_snatch_min_abs_improve
              << " snatch_rel=" << p_snatch_min_rel_improve;
    if (p_match_mode != 3) {
        std::cerr << " legacy_ts=" << p_time_safety
                  << " legacy_batch=" << p_batch_delay
                  << " local_swap=" << p_local_swap
                  << " dyn_batch=" << p_dynamic_batch;
    }
    std::cerr << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════════
// Global state
// ═══════════════════════════════════════════════════════════════════════════════

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, int> task_assignments;  // task_id → agent_id

// ═══════════════════════════════════════════════════════════════════════════════
// Runtime Statistics (5 key diagnostics)
// ═══════════════════════════════════════════════════════════════════════════════

struct ScheduleStats {
    // 1. avg_task_ticks: 平均完成一个任务所用tick数 (从分配到完成)
    //    通过 assign_tick 记录分配时刻，完成时计算差值
    std::unordered_map<int, int> assign_tick;  // task_id → 分配时的tick
    long long total_task_ticks = 0;
    int       tasks_completed  = 0;

    // 2. congestion_idx: 拥塞指数 = 所有agent执行W(wait/NA)的tick占比
    long long total_wait_ticks    = 0;
    long long total_agent_ticks   = 0;

    // 3. avg_task_dist: 完成任务的平均全程距离 (agent→start + start→...→end)
    long long total_task_dist     = 0;
    int       total_assigns       = 0;

    // 4. active_agent_pct: agent 平均启用率 (有任务的agent比例)
    long long total_active_ticks  = 0;  // 有任务的agent·tick

    // 5. busy_block_pct: 带任务agent被阻塞程度 (带任务agent中执行W的比例)
    long long busy_wait_ticks     = 0;  // 有任务但在wait的agent·tick
    long long busy_total_ticks    = 0;  // 有任务的agent·tick (same as total_active_ticks)

    // 辅助: tasks_per_1k_tick 趋势
    int       last_completed_count = 0;
    std::vector<int> tasks_per_1k;
    int       last_tick_bin     = 0;
};

static ScheduleStats g_stats;
static GamePrefilterStats g_game_prefilter_stats;

// 内部函数：导出诊断指标 JSON
static nlohmann::json export_stats_json() {
    if (g_stats.tasks_completed == 0) return nullptr;
    nlohmann::json j;
    j["tasks_completed"] = g_stats.tasks_completed;
    j["avg_task_ticks"] = g_stats.tasks_completed > 0 ?
        (double)g_stats.total_agent_ticks / g_stats.tasks_completed : 0.0;
    j["congestion_idx"] = g_stats.total_agent_ticks > 0 ?
        100.0 * g_stats.total_wait_ticks / g_stats.total_agent_ticks : 0.0;
    j["avg_task_dist"] = g_stats.total_assigns > 0 ?
        (double)g_stats.total_task_dist / g_stats.total_assigns : 0.0;
    j["active_agent_pct"] = g_stats.total_agent_ticks > 0 ?
        100.0 * g_stats.total_active_ticks / g_stats.total_agent_ticks : 0.0;
    j["busy_block_pct"] = g_stats.busy_total_ticks > 0 ?
        100.0 * g_stats.busy_wait_ticks / g_stats.busy_total_ticks : 0.0;
    return j;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Room distribution update
// ═══════════════════════════════════════════════════════════════════════════════

static void update_room_distribution(SharedEnvironment* env) {
    std::fill(agents_in_room.begin(), agents_in_room.end(), 0);
    
    for (int a = 0; a < env->num_of_agents; ++a) {
        int rid = get_room(env->curr_states.at(a).location);
        if (rid >= 0) agents_in_room[rid]++;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Reclaim
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
        if (env->curr_task_schedule[a] == -1) {
            free_agents.insert(a); sched[a] = -1;
            // Update last room for chain bonus
            if (p_chain_bonus && a < (int)agent_last_room.size()) {
                int rid = get_room(env->curr_states.at(a).location);
                if (rid >= 0) agent_last_room[a] = rid;
            }
            // Update last end location for momentum
            if (p_cost_momentum && a < (int)agent_last_end_loc.size()) {
                agent_last_end_loc[a] = env->curr_states.at(a).location;
            }
            // Update idle ticks
            if (p_idle_penalty && a < (int)agent_idle_ticks.size()) {
                agent_idle_ticks[a]++;
            }
        }
        else {
            task_assignments[env->curr_task_schedule[a]] = a;
            // Agent has task → reset idle counter
            if (p_idle_penalty && a < (int)agent_idle_ticks.size()) {
                agent_idle_ticks[a] = 0;
            }
        }
    }
    for (const auto& [tid, task] : env->task_pool) {
        if (task.agent_assigned == -1) free_tasks.insert(tid);
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
            g_stats.assign_tick.erase(tid);  // 回收的任务不计入统计
        } else { ++it; }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Reassign — release unstarted tasks that are far from agent
// ═══════════════════════════════════════════════════════════════════════════════

static void reassign_unstarted(SharedEnvironment* env, std::vector<int>& sched) {
    if (!p_reassign_enable) return;
    
    for (auto it = task_assignments.begin(); it != task_assignments.end(); ) {
        int tid = it->first, aid = it->second;
        auto ti = env->task_pool.find(tid);
        if (ti == env->task_pool.end()) { it = task_assignments.erase(it); continue; }
        
        const Task& task = ti->second;
        // Only reassign tasks that haven't been started (idx_next_loc == 0)
        if (task.idx_next_loc != 0 || task.locations.empty()) { ++it; continue; }
        
        int agent_loc = env->curr_states.at(aid).location;
        int dist = get_dist(env, agent_loc, task.locations[0]);
        
        // If agent is far from task start, release it
        if (dist > p_reassign_min_dist) {
            free_tasks.insert(tid); free_agents.insert(aid); sched[aid] = -1;
            it = task_assignments.erase(it);
            g_stats.assign_tick.erase(tid);  // reassign的任务不计入统计
        } else { ++it; }
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// Scoring function
// ═══════════════════════════════════════════════════════════════════════════════

static double compute_cost(SharedEnvironment* env, int agent_id, int task_id) {
    auto ti = env->task_pool.find(task_id);
    if (ti == env->task_pool.end()) return 1e9;
    const Task& task = ti->second;
    if (task.locations.empty()) return 1e9;

    int agent_loc = env->curr_states.at(agent_id).location;
    int task_start = task.locations[0];
    int dist_first = get_dist(env, agent_loc, task_start);

    // Time filter: skip tasks that can't be completed in remaining time
    int full_cost = task_path_cost(env, agent_loc, task);
    int remaining_ticks = p_total_ticks - env->curr_timestep;
    double effective_ts = p_time_safety;
    // Scarcity: 当task少于agents时收紧time_safety
    if (p_scarcity_ratio > 0.0) {
        int nf = (int)free_tasks.size();
        int na_local = (int)free_agents.size();
        if (na_local > 0 && nf < (int)(na_local * p_scarcity_ratio)) {
            effective_ts *= 1.2;  // 收紧20%
        }
    }
    if (full_cost * env->max_counter * effective_ts > remaining_ticks) return 1e9;

    // Distance scoring: dist_first * first_w + path_dist
    double effective_fw = p_dist_first_weight;
    // 阶段fw: 根据进度调整
    if (p_fw_phase) {
        double progress = (double)env->curr_timestep / p_total_ticks;
        if (progress < 0.33) {
            effective_fw = p_fw_early;
        } else if (progress > 0.67) {
            effective_fw = p_fw_late;
        } else {
            // 线性插值
            double t = (progress - 0.33) / 0.34;
            effective_fw = p_fw_early * (1.0 - t) + p_fw_late * t;
        }
    }
    // 动态fw: 根据task/agent比例调整
    if (p_adaptive_fw) {
        int nf = (int)free_tasks.size();
        int na_local = (int)free_agents.size();
        if (na_local > 0) {
            double ta_ratio = (double)nf / na_local;
            if (ta_ratio >= p_adaptive_fw_pivot) {
                effective_fw = p_adaptive_fw_high;  // task多→贪近
            } else {
                effective_fw = p_adaptive_fw_low;   // task少→全局优化
            }
        }
    }
    // 距离衰减fw: 近距离task低fw(避免争抢)，远距离task高fw(贪近)
    if (p_dist_decay_fw) {
        if (dist_first <= p_dist_decay_thresh) {
            effective_fw = p_dist_decay_near;
        } else {
            effective_fw = p_dist_decay_far;
        }
    }

    double total_dist = dist_first * effective_fw;
    int cur = agent_loc;
    for (size_t li = 0; li < task.locations.size(); ++li) {
        int loc = task.locations[li];
        total_dist += get_dist(env, cur, loc);
        cur = loc;
    }

    // Apply distance power (non-linear scoring)
    double cost;
    if (p_dist_pow != 1.0 && total_dist > 0) {
        cost = p_dist_overall * std::pow(total_dist, p_dist_pow);
    } else {
        cost = p_dist_overall * total_dist;
    }

    // Short task preference
    if (p_short_task_w != 0.0) {
        int internal_dist = 0;
        int c2 = task.locations[0];
        for (size_t i = 1; i < task.locations.size(); ++i) {
            internal_dist += get_dist(env, c2, task.locations[i]);
            c2 = task.locations[i];
        }
        if (p_cost_normalize && p_normalize_base > 0.0) {
            // 归一化: 避免长路径被绝对忽视，缩放到相对值
            cost += p_short_task_w * (double)internal_dist / p_normalize_base;
        } else {
            cost += p_short_task_w * internal_dist;
        }
    }

    // Chain bonus — prefer same-room task if agent just completed a task in that room
    if (p_chain_bonus && topo_ready && agent_id < (int)agent_last_room.size()) {
        int last_room = agent_last_room[agent_id];
        int task_room = get_room(task_start);
        if (last_room >= 0 && task_room >= 0 && last_room == task_room) {
            cost -= p_chain_bonus_w;
        }
    }

    // Task wait bonus: 等待越久的task cost越低(鼓励尽快分配)
    if (p_task_wait_w > 0.0) {
        int wait_ticks = env->curr_timestep - task.t_revealed;
        if (wait_ticks > p_task_wait_base) {
            cost -= p_task_wait_w * (wait_ticks - p_task_wait_base);
        }
    }

    // Priority decay: waiting longer lowers task cost, independent from legacy batch scheduling.
    if (p_priority_decay && p_priority_decay_start_ticks >= 0) {
        int wait_ticks = env->curr_timestep - task.t_revealed;
        if (wait_ticks > p_priority_decay_start_ticks) {
            double eff_decay_w = p_priority_decay_w;
            // Decay annealing: 权重随时间线性增长(前期弱衰减→后期强衰减)
            if (p_decay_annealing) {
                double progress = (double)env->curr_timestep / p_total_ticks;
                eff_decay_w = p_priority_decay_w + (p_decay_annealing_max - p_priority_decay_w) * progress;
            }
            cost -= eff_decay_w * (wait_ticks - p_priority_decay_start_ticks);
        }
    }

    // Gate congestion: 路径经过拥塞门时惩罚
    if (p_gate_cong && topo_ready && !gate_usage_count.empty()) {
        int agent_room = get_room(agent_loc);
        int task_room = get_room(task_start);
        // 如果agent和task在不同房间，路径必经门，查找最近门的拥塞
        if (agent_room >= 0 && task_room >= 0 && agent_room != task_room) {
            // 找agent到task_start路径上最可能经过的门
            int best_gate_idx = -1;
            int best_gate_dist = INT_MAX;
            for (int gi = 0; gi < (int)gates.size(); ++gi) {
                const Gate& g = gates[gi];
                if ((g.room_a == agent_room && g.room_b == task_room) ||
                    (g.room_b == agent_room && g.room_a == task_room)) {
                    int d = get_dist(env, agent_loc, g.location) + get_dist(env, g.location, task_start);
                    if (d < best_gate_dist) {
                        best_gate_dist = d;
                        best_gate_idx = gi;
                    }
                }
            }
            if (best_gate_idx >= 0 && gate_usage_count[best_gate_idx] > 3) {
                cost += p_gate_cong_w * (gate_usage_count[best_gate_idx] - 3);
            }
        }
    }

    // Lookahead chain: task终点附近有其他free_task时奖励(鼓励选"接力位"好的task)
    if (p_lookahead_chain && task.locations.size() > 0) {
        int task_end = task.locations.back();
        int nearby_count = 0;
        for (int ft : free_tasks) {
            if (ft == task_id) continue;
            auto fti = env->task_pool.find(ft);
            if (fti == env->task_pool.end() || fti->second.locations.empty()) continue;
            int d = get_dist(env, task_end, fti->second.locations[0]);
            if (d <= p_lookahead_chain_r) nearby_count++;
            if (nearby_count >= 3) break; // 最多考虑3个
        }
        if (nearby_count > 0) {
            cost -= p_lookahead_chain_w * nearby_count;
        }
    }

    // Task gravity: agent附近有多个free_task聚集时，优先让该agent工作
    if (p_task_gravity) {
        int nearby_tasks = 0;
        for (int ft : free_tasks) {
            if (ft == task_id) continue;
            auto fti = env->task_pool.find(ft);
            if (fti == env->task_pool.end() || fti->second.locations.empty()) continue;
            int d = get_dist(env, agent_loc, fti->second.locations[0]);
            if (d <= p_task_gravity_r) nearby_tasks++;
            if (nearby_tasks >= 5) break;
        }
        if (nearby_tasks >= 2) {
            cost -= p_task_gravity_w * (nearby_tasks - 1);
        }
    }

    // Cost momentum: agent上次完成task的终点与当前task起点距离近时，给cost减益
    if (p_cost_momentum && agent_id < (int)agent_last_end_loc.size()) {
        int last_end = agent_last_end_loc[agent_id];
        if (last_end >= 0) {
            int d_to_last = get_dist(env, last_end, task_start);
            if (d_to_last <= p_cost_momentum_r) {
                cost -= p_cost_momentum_w * (1.0 - (double)d_to_last / (p_cost_momentum_r + 1));
            }
        }
    }

    // Idle penalty: 长期空闲agent更容易被分配
    if (p_idle_penalty && agent_id < (int)agent_idle_ticks.size()) {
        int idle = agent_idle_ticks[agent_id];
        if (idle > p_idle_penalty_base) {
            cost -= p_idle_penalty_w * (idle - p_idle_penalty_base);
        }
    }

    // Recency bonus: 刚释放的新task给cost减益(鼓励快速响应)
    if (p_recency_bonus) {
        int age = env->curr_timestep - task.t_revealed;
        if (age <= p_recency_bonus_window) {
            cost -= p_recency_bonus_w * (1.0 - (double)age / (p_recency_bonus_window + 1));
        }
    }

    // Horizon filter: 剩余时间充裕时过滤极短task(留给后期快速冲分)
    if (p_horizon_filter) {
        int remaining = p_total_ticks - env->curr_timestep;
        if (remaining > p_horizon_filter_time) {
            int internal_dist2 = 0;
            int c3 = task.locations[0];
            for (size_t i2 = 1; i2 < task.locations.size(); ++i2) {
                internal_dist2 += get_dist(env, c3, task.locations[i2]);
                c3 = task.locations[i2];
            }
            if (internal_dist2 < p_horizon_filter_min) {
                return 1e9; // 过滤极短task
            }
        }
    }

    return cost;
}

static double compute_room0628_cost(SharedEnvironment* env, int agent_id, int task_id) {
    constexpr double kInf = 1e9;
    auto ti = env->task_pool.find(task_id);
    if (ti == env->task_pool.end()) return kInf;

    const Task& task = ti->second;
    int start_idx = std::max(0, task.idx_next_loc);
    if (start_idx >= (int)task.locations.size()) return kInf;

    const int agent_loc = env->curr_states.at(agent_id).location;
    const int first_loc = task.locations[start_idx];
    const int dist_first = get_dist(env, agent_loc, first_loc);
    if (dist_first >= INT_MAX / 8) return kInf;

    int full_path_cost = dist_first;
    int internal_dist = 0;
    int cur = first_loc;
    for (int k = start_idx + 1; k < (int)task.locations.size(); ++k) {
        const int next = task.locations[k];
        const int d = get_dist(env, cur, next);
        if (d >= INT_MAX / 8) return kInf;
        full_path_cost += d;
        internal_dist += d;
        cur = next;
    }

    const double total_dist =
        (double)dist_first * p_dist_first_weight + (double)full_path_cost;
    return p_dist_overall * total_dist + p_short_task_w * (double)internal_dist;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Hungarian algorithm (double version)
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

struct MinCostFlowEdge {
    int to;
    int rev;
    int cap;
    double cost;
};

static void add_min_cost_flow_edge(std::vector<std::vector<MinCostFlowEdge>>& graph,
                                   int from, int to, int cap, double cost) {
    MinCostFlowEdge forward{to, static_cast<int>(graph[to].size()), cap, cost};
    MinCostFlowEdge backward{from, static_cast<int>(graph[from].size()), 0, -cost};
    graph[from].push_back(forward);
    graph[to].push_back(backward);
}

static std::vector<int> solve_min_cost_assignment(const std::vector<std::vector<double>>& cost_matrix,
                                                  int required_flow) {
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

    const double inf_cost = 1e100;
    int sent_flow = 0;
    while (sent_flow < required_flow) {
        std::vector<double> dist(graph.size(), inf_cost);
        std::vector<int> prev_node(graph.size(), -1);
        std::vector<int> prev_edge(graph.size(), -1);
        std::vector<bool> in_queue(graph.size(), false);
        std::queue<int> q;
        dist[source] = 0.0;
        q.push(source);
        in_queue[source] = true;

        while (!q.empty()) {
            int node = q.front();
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

static void record_assignment_stats(SharedEnvironment* env, int agent_id, int task_id, int cur_tick) {
    auto ti = env->task_pool.find(task_id);
    if (ti == env->task_pool.end() || ti->second.locations.empty()) return;
    g_stats.assign_tick[task_id] = cur_tick;
    const int d = task_path_cost(env, env->curr_states.at(agent_id).location, ti->second);
    g_stats.total_task_dist += d;
    g_stats.total_assigns++;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Core scheduling
// ═══════════════════════════════════════════════════════════════════════════════

static void plan(SharedEnvironment* env, int time_limit, std::vector<int>& sched) {
    // TaskScheduler already applies the scheduler/planner split, matching IDEA/room-0628.
    int limit = std::max(0, time_limit);
    auto endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    int cur_tick = env->curr_timestep;

    // ---- Stats: 收集每 tick 的运行数据 ----
    int active_count = 0;  // 有任务的 agent 数
    int wait_count = 0;    // 所有 agent 中执行 wait/NA 的数
    int busy_wait = 0;     // 有任务 agent 中执行 wait/NA 的数

    for (int a = 0; a < env->num_of_agents; ++a) {
        bool has_task = (env->curr_task_schedule[a] != -1);
        bool is_waiting = (env->system_states.at(a).current_action == Action::W ||
                          env->system_states.at(a).current_action == Action::NA);
        if (has_task) {
            active_count++;
            if (is_waiting) busy_wait++;
        }
        if (is_waiting) wait_count++;
    }
    g_stats.total_agent_ticks += env->num_of_agents;
    g_stats.total_wait_ticks += wait_count;
    g_stats.total_active_ticks += active_count;
    g_stats.busy_wait_ticks += busy_wait;
    g_stats.busy_total_ticks += active_count;

    // 处理新完成的任务 → 计算 avg_task_ticks
    for (int a : env->new_freeagents) {
        // agent a 刚完成一个任务, 找到它之前的 task_id
        // 由于完成后 curr_task_schedule[a] 已经是 -1，需要从 assign_tick 中查
        // 实际上无法直接拿到 task_id，用 tasks_completed 计数即可
        g_stats.tasks_completed++;
    }

    // tasks_per_1k_tick 趋势
    int cur_bin = cur_tick / 1000;
    int new_completed = (int)env->new_freeagents.size();
    g_stats.last_completed_count += new_completed;
    while (cur_bin > g_stats.last_tick_bin) {
        g_stats.tasks_per_1k.push_back(g_stats.last_completed_count);
        g_stats.last_completed_count = 0;
        g_stats.last_tick_bin++;
    }

    // Update room distribution
    if (topo_ready) {
        update_room_distribution(env);
    }

    rebuild_state(env, sched);
    if (p_match_mode != 3) {
        reclaim_tasks(env, sched);
        reassign_unstarted(env, sched);
    }

    // Batch delay: 积累task做全局更优匹配. Mode 3 follows IDEA/room-0628 and assigns every call.
    int effective_batch = p_batch_delay;
    if (p_match_mode == 3) {
        effective_batch = 0;
    } else {
        // Phase batch: 前期batch=1快速分配，中后期batch=N
        if (p_phase_batch && p_batch_delay > 0) {
            double progress = (double)cur_tick / p_total_ticks;
            if (progress < p_phase_batch_split) {
                effective_batch = 1;  // 前期每tick分配
            }
        }
        // Dynamic batch: 根据free_tasks数量动态调batch
        if (p_dynamic_batch && !free_tasks.empty()) {
            int nft = (int)free_tasks.size();
            if (nft >= p_dynamic_batch_pivot) {
                effective_batch = p_dynamic_batch_low;   // tasks多→快速分配
            } else {
                effective_batch = p_dynamic_batch_high;  // tasks少→积累等待
            }
        }
        // Soft batch: 按free_tasks数量触发(>=min就分配, 最多等max_wait tick)
        if (p_soft_batch && !free_tasks.empty() && effective_batch > 0) {
            static int soft_batch_wait_counter = 0;
            int nft = (int)free_tasks.size();
            if (nft >= p_soft_batch_min || soft_batch_wait_counter >= p_soft_batch_max_wait) {
                effective_batch = 1;  // 立即分配
                soft_batch_wait_counter = 0;
            } else {
                soft_batch_wait_counter++;
            }
        }
    }
    if (effective_batch > 0 && cur_tick > 0 && (cur_tick % effective_batch) != 0) {
        // 非分配tick：不做新分配，仅维护状态
        for (int a : free_agents) sched[a] = -1;
        return;
    }

    // Completion-based adaptive ratio
    if (p_completion_ratio && cur_tick > 500) {
        double expected = p_completion_target * ((double)cur_tick / 1000.0);
        double actual = (double)g_stats.tasks_completed;
        double ratio_adj = actual / std::max(1.0, expected);
        if (ratio_adj < 0.9) {
            // 低于预期 → 加速(ratio提升10%)
            p_assign_ratio = std::min(1.0, p_assign_ratio * 1.1);
        } else if (ratio_adj > 1.1) {
            // 高于预期 → 保守(ratio降低5%)
            p_assign_ratio = std::max(0.3, p_assign_ratio * 0.95);
        }
    }

    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::vector<int> tasks(free_tasks.begin(), free_tasks.end());

    // Agent shuffle: 按位置哈希排序减少随机性
    if (p_agent_shuffle && agents.size() > 1) {
        std::sort(agents.begin(), agents.end(), [&](int a, int b) {
            int la = env->curr_states.at(a).location;
            int lb = env->curr_states.at(b).location;
            return la < lb;
        });
    }

    // Age sort: batch分配时按task年龄排序，优先处理久等task
    if (p_age_sort && tasks.size() > 1) {
        std::sort(tasks.begin(), tasks.end(), [&](int a, int b) {
            auto ta = env->task_pool.find(a);
            auto tb = env->task_pool.find(b);
            int age_a = (ta != env->task_pool.end()) ? (cur_tick - ta->second.t_revealed) : 0;
            int age_b = (tb != env->task_pool.end()) ? (cur_tick - tb->second.t_revealed) : 0;
            return age_a > age_b;  // 年龄大的排前面
        });
    }

    int na = agents.size(), nt = tasks.size();
    if (p_match_mode != 3 && (na == 0 || nt == 0)) { for (int a : free_agents) sched[a] = -1; return; }

    // Update gate usage count for gate congestion awareness
    if (p_gate_cong && topo_ready) {
        gate_usage_count.assign(gates.size(), 0);
        // Count how many assigned agents' paths go through each gate
        for (auto& [tid, aid] : task_assignments) {
            auto ti2 = env->task_pool.find(tid);
            if (ti2 == env->task_pool.end() || ti2->second.locations.empty()) continue;
            int aloc = env->curr_states.at(aid).location;
            int ar = get_room(aloc);
            int tr = get_room(ti2->second.locations[0]);
            if (ar >= 0 && tr >= 0 && ar != tr) {
                for (int gi = 0; gi < (int)gates.size(); ++gi) {
                    if ((gates[gi].room_a == ar && gates[gi].room_b == tr) ||
                        (gates[gi].room_b == ar && gates[gi].room_a == tr)) {
                        gate_usage_count[gi]++;
                    }
                }
            }
        }
    }

    // ═══════════════ Global assign ratio limiting ═══════════════
    int max_assign = na; // default: no cap
    double effective_ratio = p_assign_ratio;

    // Ratio phase: 前期高ratio快速吞吐，后期低ratio精选匹配
    if (p_ratio_phase) {
        double progress = (double)cur_tick / p_total_ticks;
        if (progress < p_ratio_phase_split) {
            effective_ratio = p_ratio_phase_early;
        } else {
            effective_ratio = p_ratio_phase_late;
        }
    }

    // Surplus boost: free_tasks远多于free_agents时提升ratio
    if (p_surplus_boost && na > 0 && nt > 0) {
        double task_agent_ratio = (double)nt / (double)na;
        if (task_agent_ratio > p_surplus_boost_thresh) {
            effective_ratio = std::min(1.0, effective_ratio + p_surplus_boost_ratio);
        }
    }

    if (effective_ratio < 0.999 && na > 1) {
        int total_agents = env->num_of_agents;
        int current_busy = total_agents - na;
        int global_cap = std::max(1, (int)(effective_ratio * total_agents));
        max_assign = std::max(0, global_cap - current_busy);
        if (max_assign == 0 && p_match_mode != 3) { for (int a : free_agents) sched[a] = -1; return; }
    }

    // ═══════════════ Matching with cap ═══════════════
    if (p_match_mode == 3) {
        // ---- Mode 3: IDEA/room-0628 min-cost-flow top80 stable-snatch ----
        constexpr double kInf = 1e9;

        std::unordered_set<int> candidate_agent_set(free_agents.begin(), free_agents.end());
        std::unordered_set<int> candidate_task_set(free_tasks.begin(), free_tasks.end());
        std::unordered_map<int, int> task_incumbent_agent;

        int fixed_assigned = 0;
        for (int aid = 0; aid < env->num_of_agents && aid < (int)env->curr_task_schedule.size(); ++aid) {
            const int tid = env->curr_task_schedule[aid];
            if (tid == -1) continue;

            bool can_snatch = false;
            auto ti = env->task_pool.find(tid);
            if (ti != env->task_pool.end()) {
                const Task& task = ti->second;
                if (task.idx_next_loc == 0 && !task.locations.empty()) {
                    const int pickup_dist = get_dist(env, env->curr_states.at(aid).location, task.locations.front());
                    can_snatch = pickup_dist > p_snatch_min_pickup_distance && pickup_dist < INT_MAX / 8;
                }
            }

            if (can_snatch) {
                candidate_agent_set.insert(aid);
                candidate_task_set.insert(tid);
                task_incumbent_agent[tid] = aid;
            } else {
                fixed_assigned++;
            }
        }

        for (auto it = candidate_task_set.begin(); it != candidate_task_set.end(); ) {
            auto ti = env->task_pool.find(*it);
            if (ti == env->task_pool.end() ||
                ti->second.locations.empty() ||
                ti->second.idx_next_loc < 0 ||
                ti->second.idx_next_loc >= (int)ti->second.locations.size()) {
                it = candidate_task_set.erase(it);
            } else {
                ++it;
            }
        }

        std::vector<int> candidate_agents(candidate_agent_set.begin(), candidate_agent_set.end());
        std::vector<int> candidate_tasks(candidate_task_set.begin(), candidate_task_set.end());
        std::sort(candidate_agents.begin(), candidate_agents.end());
        std::sort(candidate_tasks.begin(), candidate_tasks.end());

        auto release_unmatched_candidates = [&]() {
            for (int aid : candidate_agents) {
                int old_task = (aid >= 0 && aid < (int)env->curr_task_schedule.size())
                    ? env->curr_task_schedule[aid]
                    : -1;
                const bool was_free = old_task == -1;
                const bool was_stealable =
                    old_task != -1 &&
                    task_incumbent_agent.find(old_task) != task_incumbent_agent.end() &&
                    task_incumbent_agent[old_task] == aid;
                if (was_free || was_stealable) {
                    sched[aid] = -1;
                    if (old_task != -1) g_stats.assign_tick.erase(old_task);
                }
            }
        };

        if (candidate_agents.empty() || candidate_tasks.empty()) {
            release_unmatched_candidates();
            return;
        }

        int global_cap = std::max(1, (int)(effective_ratio * env->num_of_agents));
        int assignment_limit = std::max(0, global_cap - fixed_assigned);
        assignment_limit = std::min({assignment_limit,
                                     (int)candidate_agents.size(),
                                     (int)candidate_tasks.size()});
        if (assignment_limit <= 0) {
            release_unmatched_candidates();
            return;
        }

        std::unordered_map<int, int> candidate_agent_index;
        candidate_agent_index.reserve(candidate_agents.size());
        for (int i = 0; i < (int)candidate_agents.size(); ++i)
            candidate_agent_index[candidate_agents[i]] = i;

        std::vector<std::vector<double>> cost(candidate_agents.size(),
                                              std::vector<double>(candidate_tasks.size(), kInf));
        for (int i = 0; i < (int)candidate_agents.size(); ++i) {
            if (std::chrono::steady_clock::now() > endtime) break;
            for (int j = 0; j < (int)candidate_tasks.size(); ++j) {
                if (j % 50 == 0 && std::chrono::steady_clock::now() > endtime) break;
                const double c = compute_room0628_cost(env, candidate_agents[i], candidate_tasks[j]);
                if (c < kInf) cost[i][j] = c;
            }
        }

        for (int j = 0; j < (int)candidate_tasks.size(); ++j) {
            const int tid = candidate_tasks[j];
            auto incumbent_it = task_incumbent_agent.find(tid);
            if (incumbent_it == task_incumbent_agent.end()) continue;
            const int incumbent = incumbent_it->second;
            auto idx_it = candidate_agent_index.find(incumbent);
            if (idx_it == candidate_agent_index.end()) continue;

            const int incumbent_idx = idx_it->second;
            const double incumbent_cost = cost[incumbent_idx][j];
            if (incumbent_cost >= kInf * 0.5) continue;

            const double required_improvement =
                std::max(p_snatch_min_abs_improve,
                         std::abs(incumbent_cost) * p_snatch_min_rel_improve);
            const double max_allowed_snatch_cost = incumbent_cost - required_improvement;
            for (int i = 0; i < (int)candidate_agents.size(); ++i) {
                if (candidate_agents[i] == incumbent) continue;
                if (cost[i][j] > max_allowed_snatch_cost) cost[i][j] = kInf;
            }
        }

        std::vector<int> assignment = solve_min_cost_assignment(cost, assignment_limit);
        std::vector<std::pair<double, std::pair<int, int>>> final_pairs;
        std::unordered_set<int> planned_agents;
        std::unordered_set<int> planned_tasks;
        for (int i = 0; i < (int)assignment.size(); ++i) {
            const int j = assignment[i];
            if (j < 0 || j >= (int)candidate_tasks.size()) continue;
            if (cost[i][j] >= kInf * 0.5) continue;
            const int aid = candidate_agents[i];
            const int tid = candidate_tasks[j];
            if (planned_agents.count(aid) || planned_tasks.count(tid)) continue;
            final_pairs.push_back({cost[i][j], {aid, tid}});
            planned_agents.insert(aid);
            planned_tasks.insert(tid);
        }

        for (const auto& entry : final_pairs) {
            const int aid = entry.second.first;
            const int tid = entry.second.second;
            const int old_task = (aid >= 0 && aid < (int)env->curr_task_schedule.size())
                ? env->curr_task_schedule[aid]
                : -1;

            auto incumbent_it = task_incumbent_agent.find(tid);
            if (incumbent_it != task_incumbent_agent.end() && incumbent_it->second != aid) {
                const int incumbent = incumbent_it->second;
                if (!planned_agents.count(incumbent) &&
                    incumbent >= 0 &&
                    incumbent < (int)sched.size()) {
                    sched[incumbent] = -1;
                    g_stats.assign_tick.erase(tid);
                }
            }

            if (old_task != -1 && old_task != tid) g_stats.assign_tick.erase(old_task);
            sched[aid] = tid;
            if (old_task != tid) record_assignment_stats(env, aid, tid, cur_tick);
        }

        for (int aid : candidate_agents) {
            if (planned_agents.count(aid)) continue;
            const int old_task = (aid >= 0 && aid < (int)env->curr_task_schedule.size())
                ? env->curr_task_schedule[aid]
                : -1;
            const bool was_free = old_task == -1;
            const bool was_stealable =
                old_task != -1 &&
                task_incumbent_agent.find(old_task) != task_incumbent_agent.end() &&
                task_incumbent_agent[old_task] == aid;
            if (was_free || was_stealable) {
                sched[aid] = -1;
                if (old_task != -1) g_stats.assign_tick.erase(old_task);
            }
        }
        return;
    } else if (p_match_mode == 1) {
        // ---- Mode 1: Greedy priority queue (like GreedyHeap) ----
        // Build all (cost, agent_idx, task_idx) pairs, pop in order, cap at max_assign
        using Entry = std::tuple<double, int, int>;

        // Random engine for jitter
        static std::mt19937 rng(42);
        std::uniform_real_distribution<double> jitter_dist(0.0, p_cost_jitter);

        // Utility gate: 预计算cost均值阈值(采样)
        double utility_threshold = 1e9;
        if (p_utility_gate) {
            double sum_c = 0; int cnt_c = 0;
            int sample_a = std::min(na, 20);
            int sample_t = std::min(nt, 20);
            for (int i = 0; i < sample_a; ++i) {
                for (int j = 0; j < sample_t; ++j) {
                    double sc = compute_cost(env, agents[i], tasks[j]);
                    if (sc < 1e8) { sum_c += sc; cnt_c++; }
                }
            }
            if (cnt_c > 0) utility_threshold = (sum_c / cnt_c) * p_utility_gate_mult;
        }

        int effective_nt = nt;
        // TopK: per-agent only consider closest K tasks (by dist_first) for speed
        std::vector<std::vector<int>> agent_task_candidates;
        if (p_greedy_topk > 0 && nt > p_greedy_topk) {
            agent_task_candidates.resize(na);
            for (int i = 0; i < na; ++i) {
                int aloc = env->curr_states.at(agents[i]).location;
                std::vector<std::pair<int, int>> dist_task; // (dist, task_idx)
                dist_task.reserve(nt);
                for (int j = 0; j < nt; ++j) {
                    auto ti2 = env->task_pool.find(tasks[j]);
                    if (ti2 == env->task_pool.end() || ti2->second.locations.empty()) continue;
                    int d = get_dist(env, aloc, ti2->second.locations[0]);
                    dist_task.emplace_back(d, j);
                }
                std::partial_sort(dist_task.begin(),
                                  dist_task.begin() + std::min((int)dist_task.size(), p_greedy_topk),
                                  dist_task.end());
                int topk = std::min((int)dist_task.size(), p_greedy_topk);
                agent_task_candidates[i].reserve(topk);
                for (int k = 0; k < topk; ++k) {
                    agent_task_candidates[i].push_back(dist_task[k].second);
                }
            }
        }

        // Multi-round greedy: 第一轮分配，然后丢弃最差配对，第二轮重新分配
        int total_rounds = p_greedy_rounds;
        std::vector<std::pair<double, std::pair<int,int>>> all_assignments; // (cost, (agent, task))
        std::unordered_set<int> final_used_agents, final_used_tasks;

        for (int round = 0; round < total_rounds; ++round) {
            std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;

            for (int i = 0; i < na; ++i) {
                if (final_used_agents.count(agents[i]) && round > 0) continue; // 已确认分配的跳过
                if (std::chrono::steady_clock::now() > endtime) break;
                if (p_greedy_topk > 0 && nt > p_greedy_topk) {
                    for (int j : agent_task_candidates[i]) {
                        if (final_used_tasks.count(tasks[j]) && round > 0) continue;
                        double c = compute_cost(env, agents[i], tasks[j]);
                        if (p_cost_jitter > 0.0) c += jitter_dist(rng);
                        if (c < 1e8 && c < utility_threshold) pq.emplace(c, agents[i], tasks[j]);
                    }
                } else {
                    for (int j = 0; j < nt; ++j) {
                        if (final_used_tasks.count(tasks[j]) && round > 0) continue;
                        if (j % 50 == 0 && std::chrono::steady_clock::now() > endtime) break;
                        double c = compute_cost(env, agents[i], tasks[j]);
                        if (p_cost_jitter > 0.0) c += jitter_dist(rng);
                        if (c < 1e8 && c < utility_threshold) pq.emplace(c, agents[i], tasks[j]);
                    }
                }
            }

            // Greedy assignment
            std::vector<std::pair<double, std::pair<int,int>>> round_assignments;
            std::unordered_set<int> used_agents, used_tasks;
            // 包含已确认的
            used_agents = final_used_agents;
            used_tasks = final_used_tasks;

            int assigned_count = (int)final_used_agents.size();
            // Neighbor coordination: track which rooms have been assigned to
            std::unordered_map<int, int> room_assign_count; // room_id → count this round

            // Weighted shuffle: 从pq中取top-N候选加权随机选择(不纯贪心)
            if (p_weighted_shuffle && !pq.empty()) {
                // 提取全部候选到vector
                std::vector<Entry> all_entries;
                while (!pq.empty()) {
                    all_entries.push_back(pq.top());
                    pq.pop();
                }
                // 重新用加权随机逻辑分配
                std::uniform_real_distribution<double> u01(0.0, 1.0);
                while (!all_entries.empty() && assigned_count < max_assign && std::chrono::steady_clock::now() <= endtime) {
                    // 过滤有效候选(未分配的agent和task)
                    std::vector<int> valid_idx;
                    for (int ei = 0; ei < (int)all_entries.size(); ++ei) {
                        auto& [c, aid, tid] = all_entries[ei];
                        if (!used_agents.count(aid) && !used_tasks.count(tid) && c < 1e8) {
                            valid_idx.push_back(ei);
                        }
                    }
                    if (valid_idx.empty()) break;
                    // 取前min(10, valid)个候选计算权重
                    int pick_pool = std::min(10, (int)valid_idx.size());
                    // 按cost排序取top pick_pool
                    std::partial_sort(valid_idx.begin(), valid_idx.begin() + pick_pool, valid_idx.end(),
                        [&](int a, int b) { return std::get<0>(all_entries[a]) < std::get<0>(all_entries[b]); });
                    // 计算权重: w_i = 1 / (cost_i - cost_min + 1)^alpha
                    double min_c = std::get<0>(all_entries[valid_idx[0]]);
                    std::vector<double> weights(pick_pool);
                    double sum_w = 0;
                    for (int k = 0; k < pick_pool; ++k) {
                        double dc = std::get<0>(all_entries[valid_idx[k]]) - min_c + 1.0;
                        weights[k] = 1.0 / std::pow(dc, p_weighted_shuffle_alpha);
                        sum_w += weights[k];
                    }
                    // 加权随机选择
                    double r = u01(rng) * sum_w;
                    int chosen = 0;
                    for (int k = 0; k < pick_pool; ++k) {
                        r -= weights[k];
                        if (r <= 0) { chosen = k; break; }
                    }
                    int sel_idx = valid_idx[chosen];
                    auto [c, aid, tid] = all_entries[sel_idx];
                    round_assignments.emplace_back(c, std::make_pair(aid, tid));
                    used_agents.insert(aid);
                    used_tasks.insert(tid);
                    ++assigned_count;
                    // 标记已用
                    std::get<0>(all_entries[sel_idx]) = 1e9;
                }
            }
            // Dual queue: 先分配近程pool，再分配远程pool
            else if (p_dual_queue && !pq.empty()) {
                std::vector<Entry> near_entries, far_entries;
                while (!pq.empty()) {
                    auto [c, aid, tid] = pq.top(); pq.pop();
                    auto ti4 = env->task_pool.find(tid);
                    if (ti4 != env->task_pool.end() && !ti4->second.locations.empty()) {
                        int aloc = env->curr_states.at(aid).location;
                        int d = get_dist(env, aloc, ti4->second.locations[0]);
                        if (d < p_dual_queue_thresh) {
                            near_entries.emplace_back(c, aid, tid);
                        } else {
                            far_entries.emplace_back(c, aid, tid);
                        }
                    } else {
                        far_entries.emplace_back(c, aid, tid);
                    }
                }
                // 按cost排序近程和远程
                auto cmp = [](const Entry& a, const Entry& b) { return std::get<0>(a) < std::get<0>(b); };
                std::sort(near_entries.begin(), near_entries.end(), cmp);
                std::sort(far_entries.begin(), far_entries.end(), cmp);
                // 先分配近程
                for (auto& [c, aid, tid] : near_entries) {
                    if (assigned_count >= max_assign) break;
                    if (used_agents.count(aid) || used_tasks.count(tid)) continue;
                    round_assignments.emplace_back(c, std::make_pair(aid, tid));
                    used_agents.insert(aid);
                    used_tasks.insert(tid);
                    ++assigned_count;
                }
                // 再分配远程
                for (auto& [c, aid, tid] : far_entries) {
                    if (assigned_count >= max_assign) break;
                    if (used_agents.count(aid) || used_tasks.count(tid)) continue;
                    round_assignments.emplace_back(c, std::make_pair(aid, tid));
                    used_agents.insert(aid);
                    used_tasks.insert(tid);
                    ++assigned_count;
                }
            }
            else {
            // 标准greedy逻辑
            while (!pq.empty() && assigned_count < max_assign && std::chrono::steady_clock::now() <= endtime) {
                auto [c, aid, tid] = pq.top(); pq.pop();
                if (used_agents.count(aid) || used_tasks.count(tid)) continue;
                
                // Neighbor coordination: 同房间已有分配时加惩罚
                double adj_cost = c;
                if (p_neighbor_coord && topo_ready) {
                    auto ti3 = env->task_pool.find(tid);
                    if (ti3 != env->task_pool.end() && !ti3->second.locations.empty()) {
                        int troom = get_room(ti3->second.locations[0]);
                        if (troom >= 0 && room_assign_count[troom] > 0) {
                            adj_cost += p_neighbor_coord_w * room_assign_count[troom];
                            // 如果调整后还在pq中能找到更好选择，跳过（简化处理直接降优先级）
                        }
                    }
                }
                
                // Multi-pick: 如果次优候选cost差距小于阈值，随机选择
                if (p_multi_pick && !pq.empty()) {
                    // 看下一个同agent的候选
                    // 简化实现：直接接受当前最优
                }
                
                round_assignments.emplace_back(adj_cost, std::make_pair(aid, tid));
                used_agents.insert(aid);
                used_tasks.insert(tid);
                // 更新房间计数
                if (p_neighbor_coord && topo_ready) {
                    auto ti3 = env->task_pool.find(tid);
                    if (ti3 != env->task_pool.end() && !ti3->second.locations.empty()) {
                        int troom = get_room(ti3->second.locations[0]);
                        if (troom >= 0) room_assign_count[troom]++;
                    }
                }
                ++assigned_count;
            }
            } // end else (standard greedy)

            if (round < total_rounds - 1 && !round_assignments.empty()) {
                // 丢弃最差的 drop_pct 配对，确认其余
                std::sort(round_assignments.begin(), round_assignments.end());
                int keep_count = std::max(1, (int)(round_assignments.size() * (1.0 - p_greedy_drop_pct)));
                for (int k = 0; k < keep_count; ++k) {
                    auto [c2, p2] = round_assignments[k];
                    final_used_agents.insert(p2.first);
                    final_used_tasks.insert(p2.second);
                    all_assignments.push_back(round_assignments[k]);
                }
                // 丢弃的不加入 final
            } else {
                // 最后一轮：全部确认
                for (auto& entry : round_assignments) {
                    all_assignments.push_back(entry);
                }
            }
        }

        // Confidence gate: 只保留确定性高的配对，不确定的释放给后续优化
        if (p_confidence_gate && (int)all_assignments.size() >= 2 && std::chrono::steady_clock::now() <= endtime) {
            // Adaptive confidence: 根据free_tasks数量动态调节阈值
            double eff_conf_thresh = p_confidence_thresh;
            if (p_adaptive_conf) {
                int nft = (int)free_tasks.size();
                if (nft >= p_adaptive_conf_pivot) {
                    eff_conf_thresh = p_adaptive_conf_low;   // task多→宽松
                } else {
                    eff_conf_thresh = p_adaptive_conf_high;  // task少→严格
                }
            }
            // 对每个配对计算: 该agent对其他已分配task的次优cost差距
            std::vector<bool> keep_flag(all_assignments.size(), true);
            for (int idx = 0; idx < (int)all_assignments.size(); ++idx) {
                int aid = all_assignments[idx].second.first;
                int tid = all_assignments[idx].second.second;
                double my_cost = all_assignments[idx].first;
                // 找同agent对其他task的次优cost
                double second_best = 1e9;
                for (int jdx = 0; jdx < (int)all_assignments.size(); ++jdx) {
                    if (jdx == idx) continue;
                    int other_tid = all_assignments[jdx].second.second;
                    double alt = compute_cost(env, aid, other_tid);
                    if (alt < second_best) second_best = alt;
                }
                double gap = second_best - my_cost;
                // 如果差距小于阈值→不确定，标记为释放
                if (gap < eff_conf_thresh && gap >= 0) {
                    keep_flag[idx] = false;
                }
            }
            // 释放不确定的配对（最多释放30%，避免全释放）
            int released = 0;
            int max_release = std::max(1, (int)(all_assignments.size() * 0.3));
            std::vector<std::pair<double, std::pair<int,int>>> kept;
            for (int idx = 0; idx < (int)all_assignments.size(); ++idx) {
                if (!keep_flag[idx] && released < max_release) {
                    released++;
                } else {
                    kept.push_back(all_assignments[idx]);
                }
            }
            all_assignments = kept;
        }

        // Regret-based refinement: 对regret最小的配对释放重分配
        if (p_regret_mode && all_assignments.size() > 1 && std::chrono::steady_clock::now() <= endtime) {
            // 计算每个已分配agent的regret = 次优cost - 最优cost
            // regret小说明该agent分配到当前task的优势不明显，可能有更好的分配方式
            std::vector<std::pair<double, int>> regrets; // (regret, idx in all_assignments)
            for (int idx = 0; idx < (int)all_assignments.size(); ++idx) {
                int aid = all_assignments[idx].second.first;
                int tid = all_assignments[idx].second.second;
                double best_cost = all_assignments[idx].first;
                // 找次优: 该agent对其他未分配task的最低cost
                double second_best = 1e9;
                for (auto& [c2, p2] : all_assignments) {
                    if (p2.second == tid) continue;
                    double alt = compute_cost(env, aid, p2.second);
                    if (alt < second_best) second_best = alt;
                }
                double regret = second_best - best_cost;
                if (regret < p_regret_thresh) {
                    regrets.emplace_back(regret, idx);
                }
            }
            // 对regret最小的配对(最多释放20%)尝试重分配
            if (!regrets.empty()) {
                std::sort(regrets.begin(), regrets.end());
                int release_count = std::max(1, (int)(regrets.size() * 0.2));
                release_count = std::min(release_count, (int)regrets.size());
                std::unordered_set<int> released_idx;
                for (int k = 0; k < release_count; ++k) {
                    released_idx.insert(regrets[k].second);
                }
                // 收集释放的agents和tasks
                std::vector<int> re_agents, re_tasks;
                for (int idx : released_idx) {
                    re_agents.push_back(all_assignments[idx].second.first);
                    re_tasks.push_back(all_assignments[idx].second.second);
                }
                // 贪心重分配释放的agents和tasks
                std::priority_queue<Entry, std::vector<Entry>, std::greater<>> re_pq;
                for (int a : re_agents) {
                    for (int t : re_tasks) {
                        double c = compute_cost(env, a, t);
                        if (c < 1e8) re_pq.emplace(c, a, t);
                    }
                }
                std::unordered_set<int> re_used_a, re_used_t;
                std::vector<std::pair<double, std::pair<int,int>>> new_assigns;
                while (!re_pq.empty()) {
                    auto [c, aid2, tid2] = re_pq.top(); re_pq.pop();
                    if (re_used_a.count(aid2) || re_used_t.count(tid2)) continue;
                    new_assigns.emplace_back(c, std::make_pair(aid2, tid2));
                    re_used_a.insert(aid2);
                    re_used_t.insert(tid2);
                }
                // 替换原分配
                for (int idx : released_idx) {
                    all_assignments[idx].first = 1e9; // mark invalid
                }
                for (auto& entry : new_assigns) {
                    all_assignments.push_back(entry);
                }
                // 清除invalid
                all_assignments.erase(
                    std::remove_if(all_assignments.begin(), all_assignments.end(),
                        [](const auto& e) { return e.first >= 1e9; }),
                    all_assignments.end());
            }
        }

        // Local swap (2-opt): 对最差K个配对尝试交换，改善匹配质量
        if (p_local_swap && (int)all_assignments.size() >= 2 && std::chrono::steady_clock::now() <= endtime) {
            // 按cost降序排列(最差在前)
            std::sort(all_assignments.begin(), all_assignments.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            int check_count = std::min(p_local_swap_k, (int)all_assignments.size());
            bool improved = true;
            int max_iters = 3; // 最多迭代3轮
            while (improved && max_iters-- > 0 && std::chrono::steady_clock::now() <= endtime) {
                improved = false;
                for (int i = 0; i < check_count && std::chrono::steady_clock::now() <= endtime; ++i) {
                    for (int j = i + 1; j < (int)all_assignments.size(); ++j) {
                        int a1 = all_assignments[i].second.first;
                        int t1 = all_assignments[i].second.second;
                        int a2 = all_assignments[j].second.first;
                        int t2 = all_assignments[j].second.second;
                        double old_cost = all_assignments[i].first + all_assignments[j].first;
                        // 尝试交换: a1→t2, a2→t1
                        double c1_new = compute_cost(env, a1, t2);
                        double c2_new = compute_cost(env, a2, t1);
                        if (c1_new < 1e8 && c2_new < 1e8 && (c1_new + c2_new) < old_cost) {
                            all_assignments[i] = {c1_new, {a1, t2}};
                            all_assignments[j] = {c2_new, {a2, t1}};
                            improved = true;
                        }
                    }
                }
                // 重新排序
                if (improved) {
                    std::sort(all_assignments.begin(), all_assignments.end(),
                              [](const auto& a, const auto& b) { return a.first > b.first; });
                }
            }
        }

        // Bipartite refine: 对最差K个配对用小规模匈牙利精确优化
        if (p_bipartite_refine && (int)all_assignments.size() >= 2 && std::chrono::steady_clock::now() <= endtime) {
            std::sort(all_assignments.begin(), all_assignments.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            int refine_k = std::min(p_bipartite_refine_k, (int)all_assignments.size());
            // 提取最差k个配对的agents和tasks
            std::vector<int> ref_agents, ref_tasks;
            ref_agents.reserve(refine_k);
            ref_tasks.reserve(refine_k);
            for (int i = 0; i < refine_k; ++i) {
                ref_agents.push_back(all_assignments[i].second.first);
                ref_tasks.push_back(all_assignments[i].second.second);
            }
            // 构建小规模cost矩阵并用匈牙利求解
            int rn = refine_k;
            std::vector<std::vector<double>> ref_cost(rn, std::vector<double>(rn, 1e9));
            for (int i = 0; i < rn; ++i) {
                for (int j = 0; j < rn; ++j) {
                    ref_cost[i][j] = compute_cost(env, ref_agents[i], ref_tasks[j]);
                }
            }
            std::vector<int> ref_assign;
            hungarian(ref_cost, ref_assign);
            // 比较总cost是否改善
            double old_total = 0, new_total = 0;
            bool valid = true;
            for (int i = 0; i < rn; ++i) {
                old_total += all_assignments[i].first;
                if (ref_assign[i] >= 0 && ref_assign[i] < rn && ref_cost[i][ref_assign[i]] < 1e8) {
                    new_total += ref_cost[i][ref_assign[i]];
                } else {
                    valid = false;
                    break;
                }
            }
            if (valid && new_total < old_total) {
                for (int i = 0; i < rn; ++i) {
                    all_assignments[i] = {ref_cost[i][ref_assign[i]],
                                          {ref_agents[i], ref_tasks[ref_assign[i]]}};
                }
            }
        }

        // Apply final assignments
        for (auto& [c, p2] : all_assignments) {
            int aid = p2.first, tid = p2.second;
            sched[aid] = tid;
            free_agents.erase(aid);
            free_tasks.erase(tid);
            // Stats
            auto ti2 = env->task_pool.find(tid);
            if (ti2 != env->task_pool.end() && !ti2->second.locations.empty()) {
                g_stats.assign_tick[tid] = cur_tick;
                int d = task_path_cost(env, env->curr_states.at(aid).location, ti2->second);
                g_stats.total_task_dist += d;
                g_stats.total_assigns++;
            }
        }
    } else if (p_match_mode == 2) {
        // ---- Mode 2: Full Hungarian + TopK selection ----
        // Run hungarian on ALL free agents × tasks, then pick top max_assign by matched cost
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

        // Collect valid matches with their costs
        std::vector<std::tuple<double, int, int>> matches; // (cost, agent_idx, task_idx)
        for (int i = 0; i < na; ++i) {
            if (assign[i] >= 0 && assign[i] < nt && cost[i][assign[i]] < 1e8) {
                matches.emplace_back(cost[i][assign[i]], i, assign[i]);
            }
        }
        // Sort by cost ascending, keep only top max_assign
        std::sort(matches.begin(), matches.end());
        int to_assign = std::min((int)matches.size(), max_assign);
        for (int k = 0; k < (int)matches.size(); ++k) {
            auto [c, ai, ti] = matches[k];
            if (k < to_assign) {
                sched[agents[ai]] = tasks[ti];
                free_agents.erase(agents[ai]);
                free_tasks.erase(tasks[ti]);
            } else {
                sched[agents[ai]] = -1;
            }
        }
    } else {
        // ---- Mode 0 (default): Truncate by distance, then Hungarian ----
        if (na > max_assign) {
            std::vector<std::pair<int, int>> scored;
            scored.reserve(na);
            for (int aid : agents) {
                int aloc = env->curr_states.at(aid).location;
                int min_d = INT_MAX;
                for (int j = 0; j < std::min(nt, 20); ++j) {
                    auto it = env->task_pool.find(tasks[j]);
                    if (it == env->task_pool.end() || it->second.locations.empty()) continue;
                    int d = get_dist(env, aloc, it->second.locations[0]);
                    min_d = std::min(min_d, d);
                }
                scored.emplace_back(min_d, aid);
            }
            std::sort(scored.begin(), scored.end());
            std::vector<int> active_agents;
            active_agents.reserve(max_assign);
            for (int i = 0; i < (int)scored.size(); ++i) {
                if (i < max_assign) {
                    active_agents.push_back(scored[i].second);
                } else {
                    sched[scored[i].second] = -1;
                }
            }
            agents.swap(active_agents);
            na = agents.size();
        }
        if (na == 0) { for (int a : free_agents) sched[a] = -1; return; }

        // Hungarian matching
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

        // IDEA7: Post-Hungarian swap optimization
        if (p_post_swap && na >= 4) {
            // Find worst 20% of assignments, try pairwise swap
            std::vector<std::pair<double, int>> assigned_costs;
            for (int i = 0; i < na; ++i) {
                if (assign[i] >= 0 && assign[i] < nt && cost[i][assign[i]] < 1e8) {
                    assigned_costs.emplace_back(cost[i][assign[i]], i);
                }
            }
            std::sort(assigned_costs.rbegin(), assigned_costs.rend());
            int swap_count = std::min((int)assigned_costs.size() / 5, 10);
            for (int s = 0; s < swap_count && s < (int)assigned_costs.size(); ++s) {
                int i1 = assigned_costs[s].second;
                int j1 = assign[i1];
                if (j1 < 0) continue;
                // Try swap with a random better assignment
                for (int s2 = swap_count; s2 < (int)assigned_costs.size() && s2 < swap_count + 5; ++s2) {
                    int i2 = assigned_costs[s2].second;
                    int j2 = assign[i2];
                    if (j2 < 0 || i1 == i2) continue;
                    double old_cost = cost[i1][j1] + cost[i2][j2];
                    double new_cost = cost[i1][j2] + cost[i2][j1];
                    if (new_cost < old_cost - 0.1) {
                        // Swap
                        sched[agents[i1]] = tasks[j2];
                        sched[agents[i2]] = tasks[j1];
                        assign[i1] = j2;
                        assign[i2] = j1;
                        break;
                    }
                }
            }
        }
    }

    for (int a : free_agents) sched[a] = -1;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Initialize & entry
// ═══════════════════════════════════════════════════════════════════════════════

static void do_init(SharedEnvironment* env, int preprocess_time_limit) {
    free_agents.clear(); free_tasks.clear(); task_assignments.clear();
    agent_last_room.assign(env->num_of_agents, -1);
    agent_last_end_loc.assign(env->num_of_agents, -1);
    agent_idle_ticks.assign(env->num_of_agents, 0);
    g_stats = ScheduleStats{};  // reset
    load_params();
    const int schedule_init_limit = std::max(
        1, preprocess_time_limit - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE - 1000);
    DefaultPlanner::schedule_initialize(schedule_init_limit, env);
    pht_initialize(env);
    topo_initialize(env);
    std::cerr << "[OptRoom] initialized (topo_ready=" << topo_ready << ")\n";
}

static void do_plan(SharedEnvironment* env, int time_limit, std::vector<int>& sched) {
    plan(env, time_limit, sched);
}

} // namespace optroom_internal

// ═══════════════════════════════════════════════════════════════════════════════
// Public interface
// ═══════════════════════════════════════════════════════════════════════════════

void OptRoomScheduler::initialize(SharedEnvironment* env, int preprocess_time_limit_ms) {
    optroom_internal::do_init(env, preprocess_time_limit_ms);
}

void OptRoomScheduler::schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env) {
    optroom_internal::do_plan(env, time_limit_ms, proposed_schedule);
}

nlohmann::json get_schedule_stats_json() {
    return optroom_internal::export_stats_json();
}

void set_schedule_result_summary(int /*num_task_finished*/, int /*final_tick*/) {}

void record_schedule_task_progress(int /*timestep*/, int /*cumulative_finished*/) {}

void set_game_prefilter_stats(const GamePrefilterStats& stats) {
    optroom_internal::g_game_prefilter_stats = stats;
}

GamePrefilterStats get_game_prefilter_stats() {
    return optroom_internal::g_game_prefilter_stats;
}

} // namespace orz_engine
