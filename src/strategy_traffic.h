// strategy_traffic.h — Traffic-Aware Batch Scheduling
// 全新独立策略，不依赖 strategy_centerline.h
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "traffic_config.h"
#include "centerline.h"
#include "orz_highway.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <random>

namespace strategy_traffic {
// ====== 距离工具（独立于 strategy_centerline） ======
static constexpr float TRAFFIC_COEFF_AGENT_TASK1 = 1.0f;
static constexpr float TRAFFIC_COEFF_TASK_SEGMENT = 0.5f;

static inline int unified_dist(int loc_a, int loc_b) {
    int d = BFSTable::get_dist(loc_a, loc_b);
    if (d > 0) return d;
    int a_m, a_b, b_m, b_b;
    Centerline::project(loc_a, a_m, a_b);
    Centerline::project(loc_b, b_m, b_b);
    if (a_m >= 0 && b_m >= 0)
        return std::abs(b_m - a_m) + std::abs(b_b - a_b);
    return std::abs(loc_a % Centerline::g_cols - loc_b % Centerline::g_cols) +
           std::abs(loc_a / Centerline::g_cols - loc_b / Centerline::g_cols);
}


// ====== 全局状态（init 时初始化） ======
static bool g_inited = false;
static std::vector<bool> g_mp_highway_flag;  // [si] = 标注格对应的 main point
static int g_highway_mp_count = 0;

// ====== 工具函数 ======
inline int zid(int loc, int cols) {
    return (loc / cols / 24) * Centerline::g_n24c + (loc % cols / 24);
}

// ====== Init ======
void init(SharedEnvironment* env) {
    Centerline::init(env);
    ORZ_Highway::init();
    g_mp_highway_flag.assign(Centerline::g_mp_count, false);
    g_highway_mp_count = 0;

    // 遍历所有 forbid/guide 格 → 投影到 main point
    for (int i = 0; i < ORZ_Highway::NUM_FORBID; i++) {
        int loc = ORZ_Highway::kForbid[i].loc;
        int m, b;
        Centerline::project(loc, m, b);
        if (m < 0) continue;
        int si = (int)(std::lower_bound(Centerline::g_mp_main_dist.begin(),
            Centerline::g_mp_main_dist.end(), m) - Centerline::g_mp_main_dist.begin());
        if (si >= 0 && si < Centerline::g_mp_count && !g_mp_highway_flag[si]) {
            g_mp_highway_flag[si] = true;
            g_highway_mp_count++;
        }
    }
    for (int i = 0; i < ORZ_Highway::NUM_GUIDE; i++) {
        int loc = ORZ_Highway::kGuide[i].loc;
        int m, b;
        Centerline::project(loc, m, b);
        if (m < 0) continue;
        int si = (int)(std::lower_bound(Centerline::g_mp_main_dist.begin(),
            Centerline::g_mp_main_dist.end(), m) - Centerline::g_mp_main_dist.begin());
        if (si >= 0 && si < Centerline::g_mp_count && !g_mp_highway_flag[si]) {
            g_mp_highway_flag[si] = true;
            g_highway_mp_count++;
        }
    }

    g_inited = true;
}

// ====== 标注 main point 的 future flow ======
// 只给 g_mp_highway_flag 为 true 的 main point 加流
void add_highway_future_flow(int agent_loc, const std::vector<int>& task_locs,
                              SharedEnvironment* env, bool do_instant_lock) {
    if (!g_inited || g_highway_mp_count == 0) return;

    int m_curr, b_curr;
    Centerline::project(agent_loc, m_curr, b_curr);
    if (m_curr < 0) return;

    int prev_m = m_curr;
    int prev_loc = agent_loc;
    int cumul_si = 0;

    for (int loc : task_locs) {
        int tm, tb;
        Centerline::project(loc, tm, tb);
        if (tm < 0) continue;

        // 分支路径：不经过 main point，跳过
        if (BFSTable::is_branch_path(prev_loc, loc)) {
            cumul_si += std::abs(tm - prev_m);
            prev_m = tm;
            prev_loc = loc;
            continue;
        }

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int dir = (tm > prev_m) ? +1 : -1;

        auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(),
                                       Centerline::g_mp_main_dist.end(), lo);
        auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(),
                                       Centerline::g_mp_main_dist.end(), hi);

        for (int si = (int)(lo_it - Centerline::g_mp_main_dist.begin());
             si < (int)(hi_it - Centerline::g_mp_main_dist.begin()); si++) {

            if (!g_mp_highway_flag[si]) continue;  // 只算标注点

            int sm = Centerline::g_mp_main_dist[si];
            if ((dir > 0 && sm <= prev_m) || (dir < 0 && sm >= prev_m)) continue;

            cumul_si += std::abs(sm - prev_m);
            float decay = 1.0f / (1.0f + (float)cumul_si * FF_DECAY_K);

            if (dir > 0)
                Centerline::g_mp_future_right[si] += decay;
            else
                Centerline::g_mp_future_left[si] += decay;
        }
        prev_m = tm;
        prev_loc = loc;
    }
}

// ====== 计算全局平均流量 ======
float compute_avg_flow() {
    if (g_highway_mp_count == 0) return 0;
    float total = 0;
    for (int si = 0; si < Centerline::g_mp_count; si++) {
        if (!g_mp_highway_flag[si]) continue;
        total += Centerline::g_mp_future_right[si] + Centerline::g_mp_future_left[si];
    }
    return total / (float)g_highway_mp_count;
}

// ====== Pre-tick updates ======
void pre_tick_updates(SharedEnvironment* env) {
    if (!g_inited) return;

    // 1. 清空 future flow（仅标注点）
    for (int si = 0; si < Centerline::g_mp_count; si++) {
        if (g_mp_highway_flag[si]) {
            Centerline::g_mp_future_right[si] = 0;
            Centerline::g_mp_future_left[si] = 0;
        }
    }

    // 2. 已有任务的 agent 加流
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        add_highway_future_flow(env->curr_states[a].location,
                                it->second.locations, env, false);
    }

    // 3. Idle agent 加流回 spawn
    // (保留原有 idle 回流逻辑)
}

// ====== Base Score ======
float base_score(int a_loc, int tid, SharedEnvironment* env, float avg_flow) {
    auto it = env->task_pool.find(tid);
    if (it == env->task_pool.end() || it->second.locations.empty())
        return -1e9f;

    const auto& locs = it->second.locations;

    // ---- Phase 1: Branch exemption ----
    {
        bool is_branch = true;
        int check_prev = a_loc;
        for (size_t ei = 0; ei < locs.size(); ei++) {
            if (!BFSTable::is_branch_path(check_prev, locs[ei])) {
                is_branch = false;
                break;
            }
            check_prev = locs[ei];
        }
        if (is_branch) {
            float wd = 0;
            int prev = a_loc;
            for (size_t ei = 0; ei < locs.size(); ei++) {
                int d = unified_dist(prev, locs[ei]);
                float coeff = (ei == 0) ? TRAFFIC_COEFF_AGENT_TASK1 : TRAFFIC_COEFF_TASK_SEGMENT;
                wd += (float)d * coeff;
                prev = locs[ei];
            }
            return 1.0f / (wd + 1.0f);
        }
    }

    // ---- Phase 2: Main line traffic-aware scoring ----
    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    float weighted_dist = 0;
    int prev_m = a_m;
    int prev_loc = a_loc;
    float penalty = 0;

    for (size_t ei = 0; ei < locs.size(); ei++) {
        int tm, tb;
        Centerline::project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int d_seg = unified_dist(prev_loc, locs[ei]);
        float coeff = (ei == 0) ? TRAFFIC_COEFF_AGENT_TASK1 : TRAFFIC_COEFF_TASK_SEGMENT;
        weighted_dist += (float)d_seg * coeff;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(),
                                       Centerline::g_mp_main_dist.end(), lo);
        auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(),
                                       Centerline::g_mp_main_dist.end(), hi);

        for (int si = (int)(lo_it - Centerline::g_mp_main_dist.begin());
             si < (int)(hi_it - Centerline::g_mp_main_dist.begin()); si++) {

            if (!g_mp_highway_flag[si]) continue;

            float fr = Centerline::g_mp_future_right[si];
            float fl = Centerline::g_mp_future_left[si];
            float ftotal = fr + fl;

            // ① 全局均衡：高于平均就扣
            if (avg_flow > 0 && ftotal > avg_flow) {
                float overload = (ftotal - avg_flow) / avg_flow;
                penalty += overload * FF_GLOBAL_W;
            }

            // ② 局部均衡：方向失衡
            if (fr + fl > 0) {
                float dir_balance = (fr - fl) / (fr + fl);        // [-1,+1]
                float direction_score = (float)intent * dir_balance;
                penalty += direction_score * FF_DIR_W;
            }
        }
        prev_m = tm;
        prev_loc = locs[ei];
    }

    float baseline = 1.0f / (weighted_dist + 1.0f);
    return baseline / (1.0f + penalty);
}

// ====== 从不同列随机挑 agent ======
std::vector<int> pick_diverse_agents(const std::vector<int>& pool, int n,
                                      SharedEnvironment* env) {
    if (pool.empty() || n <= 0) return {};

    int cols_per = env->cols / BM_COL_SEGMENTS + 1;
    std::vector<int> buckets[BM_COL_SEGMENTS];
    for (int a : pool) {
        int col = env->curr_states[a].location % env->cols;
        int b = col / cols_per;
        if (b >= BM_COL_SEGMENTS) b = BM_COL_SEGMENTS - 1;
        buckets[b].push_back(a);
    }

    static std::mt19937 rng((std::random_device())());
    std::vector<int> result;
    int tries = 0;
    while ((int)result.size() < n && tries < n * 3) {
        tries++;
        int b = rng() % BM_COL_SEGMENTS;
        if (buckets[b].empty()) continue;
        int idx = rng() % (int)buckets[b].size();
        result.push_back(buckets[b][idx]);
        buckets[b].erase(buckets[b].begin() + idx);
    }
    return result;
}

// ====== Plan ======
void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    auto t0 = std::chrono::steady_clock::now();
    TimePoint endtime = t0 + std::chrono::milliseconds(limit);

    if (!g_inited) init(env);

    // 1. Pre-tick: future flow + avg_flow
    pre_tick_updates(env);
    float avg_flow = compute_avg_flow();

    // 2. 收集 free agent 和 free task
    std::vector<int> free_agents;
    std::unordered_set<int> free_tasks;
    std::unordered_set<int> assigned_tasks;

    for (int a = 0; a < env->num_of_agents; a++) {
        int cur = env->curr_task_schedule[a];
        if (cur < 0) free_agents.push_back(a);
        else assigned_tasks.insert(cur);
    }
    for (int tid : env->new_tasks) free_tasks.insert(tid);
    for (auto& kv : env->task_pool) {
        if (!assigned_tasks.count(kv.first)) free_tasks.insert(kv.first);
    }

    static std::mt19937 rng((std::random_device())());

    // 3. Phase 1: Branch tasks first (no main-line traffic)
    for (auto it = free_agents.begin(); it != free_agents.end();) {
        int a = *it;
        int a_loc = env->curr_states[a].location;
        int best_tid = -1;
        float best_score = -1e9f;

        for (int tid : free_tasks) {
            auto tk = env->task_pool.find(tid);
            if (tk == env->task_pool.end()) continue;

            // 检查是否纯 branch
            bool is_branch = true;
            int cp = a_loc;
            for (int loc : tk->second.locations) {
                if (!BFSTable::is_branch_path(cp, loc)) { is_branch = false; break; }
                cp = loc;
            }
            if (!is_branch) continue;

            float wd = 0;
            int prev = a_loc;
            for (size_t ei = 0; ei < tk->second.locations.size(); ei++) {
                int d = unified_dist(prev, tk->second.locations[ei]);
                wd += (float)d * ((ei == 0) ? TRAFFIC_COEFF_AGENT_TASK1 : TRAFFIC_COEFF_TASK_SEGMENT);
                prev = tk->second.locations[ei];
            }
            float s = 1.0f / (wd + 1.0f);
            if (s > best_score) { best_score = s; best_tid = tid; }
        }

        if (best_tid >= 0) {
            proposed_schedule[a] = best_tid;
            auto tk = env->task_pool.find(best_tid);
            if (tk != env->task_pool.end())
                add_highway_future_flow(a_loc, tk->second.locations, env, false);
            free_tasks.erase(best_tid);
            it = free_agents.erase(it);
        } else {
            ++it;
        }
    }

    // 4. Phase 2: Batch matching
    // 提前过滤标注格上的任务（只过滤一次，后面不用再检查）
    {
        std::unordered_set<int> to_remove;
        for (int tid : free_tasks) {
            auto tk = env->task_pool.find(tid);
            if (tk == env->task_pool.end()) continue;
            for (int loc : tk->second.locations) {
                if (ORZ_Highway::is_highway_cell(loc)) {
                    to_remove.insert(tid);
                    break;
                }
            }
        }
        for (int tid : to_remove)
            free_tasks.erase(tid);
    }

    // 计算活跃率上限 + zone 映射
    int total_agents = env->num_of_agents;
    int max_active = (int)(total_agents * BM_MAX_ACTIVE_RATIO);
    int cur_active = total_agents - (int)free_agents.size();

    // 预计算每个 free agent 和 free task 的 zone
    std::unordered_map<int, int> agent_zone_map;
    for (int a : free_agents) {
        int a_loc = env->curr_states[a].location;
        int a_m, a_b;
        Centerline::project(a_loc, a_m, a_b);
        agent_zone_map[a] = (a_m >= 0) ? a_m / TRAFFIC_ZONE_STEP : -1;
    }
    std::unordered_map<int, int> task_zone_map;
    for (int tid : free_tasks) {
        auto it = env->task_pool.find(tid);
        if (it != env->task_pool.end() && !it->second.locations.empty()) {
            int tm, tb;
            Centerline::project(it->second.locations[0], tm, tb);
            task_zone_map[tid] = (tm >= 0) ? tm / TRAFFIC_ZONE_STEP : -1;
        }
    }

    int batch_size = BM_BATCH_SIZE;
    if (batch_size > (int)free_agents.size()) batch_size = (int)free_agents.size();

    while (!free_agents.empty() && !free_tasks.empty() &&
           cur_active < max_active &&
           std::chrono::steady_clock::now() <= endtime) {

        auto batch = pick_diverse_agents(free_agents, batch_size, env);
        if (batch.empty()) break;

        int alloc_target = std::max(1, (int)(batch.size() * BM_ALLOC_RATIO));
        int batch_allocated = 0;
        std::unordered_set<int> batch_done_a, batch_done_t;

        // 每分配一个 agent，重算全部剩余 agent × 全部剩余 task 的分
        while (batch_allocated < alloc_target && batch_done_a.size() < batch.size()) {
            int best_a = -1, best_tid = -1;
            float best_score = -1e9f;

            for (int a : batch) {
                if (batch_done_a.count(a)) continue;
                int a_loc = env->curr_states[a].location;
                int a_zone = agent_zone_map.count(a) ? agent_zone_map[a] : -1;
                for (int tid : free_tasks) {
                    if (batch_done_t.count(tid)) continue;
                    // zone 范围过滤（无法投影的不限）
                    int t_zone = task_zone_map.count(tid) ? task_zone_map[tid] : -1;
                    if (a_zone >= 0 && t_zone >= 0 &&
                        std::abs(a_zone - t_zone) > TRAFFIC_ZONE_RANGE)
                        continue;
                    float s = base_score(a_loc, tid, env, avg_flow);
                    if (s > best_score) {
                        best_score = s;
                        best_a = a;
                        best_tid = tid;
                    }
                }
            }

            if (best_a < 0 || best_score <= 0) break;  // 无可用分配

            // 分配最佳 pair
            proposed_schedule[best_a] = best_tid;
            batch_done_a.insert(best_a);
            batch_done_t.insert(best_tid);
            batch_allocated++;
            cur_active++;

            // 实时更新 flow + avg_flow
            int a_loc = env->curr_states[best_a].location;
            auto tk = env->task_pool.find(best_tid);
            if (tk != env->task_pool.end())
                add_highway_future_flow(a_loc, tk->second.locations, env, false);
            avg_flow = compute_avg_flow();
        }

        // 从池中移除已分配
        for (int a : batch_done_a)
            free_agents.erase(std::remove(free_agents.begin(), free_agents.end(), a),
                              free_agents.end());
        for (int tid : batch_done_t)
            free_tasks.erase(tid);

        // shuffle 其余
        std::shuffle(free_agents.begin(), free_agents.end(), rng);
    }

    // 5. 剩下的设为 idle
    for (int a : free_agents)
        proposed_schedule[a] = -1;
}

} // namespace strategy_traffic
