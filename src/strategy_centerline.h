// strategy_centerline.h — Centerline M+B makespan greedy scheduler
// Uses Centerline::project() for distance along corridor.
// Init done in preprocess phase. Pure greedy, no congestion/bonuses.

#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cstdint>
#include <chrono>
#include <climits>
#include <cmath>

namespace strategy_centerline {

// ====== Traffic constants ======
static constexpr float COLD_WEIGHT = 0.0f;
static constexpr float HOT_WEIGHT = 0.002f;
static constexpr float ROUTE_WEIGHT = 0.002f;
static constexpr float DIR_MATCH_WEIGHT = 0.003f;
static constexpr float CROSS_WEIGHT = 0.010f;
static constexpr int COLD_THRESHOLD = 15;
static constexpr int HOT_THRESHOLD = 40;

// ====== Zone-at-main mapping (main_dist -> zone24 index) ======
static std::vector<int> g_zone_at_main;

inline void build_zone_at_main(const SharedEnvironment* env) {
    int cols = env->cols;
    int n24c = (cols + 23) / 24;
    g_zone_at_main.assign(Centerline::g_total_len + 1, -1);
    for (int loc : Centerline::g_centerline_cells) {
        if (loc < 0) continue;
        int m = Centerline::g_main_dist[loc];
        if (m >= 0 && m <= Centerline::g_total_len) {
            int r = loc / cols, c = loc % cols;
            g_zone_at_main[m] = (r / 24) * n24c + (c / 24);
        }
    }
}

// ====== State ======
static SharedEnvironment* g_env = nullptr;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
static inline int zh_dist(int z1, int z2) {
    return std::abs(z1/100 - z2/100) + std::abs(z1%100 - z2%100);
}

// ====== Init: all heavy work in preprocess ======
void init(SharedEnvironment* env) {
    g_env = env;
    Centerline::init(env);
    build_zone_at_main(env);
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d\n",
           env->num_of_agents, env->cols, env->rows);
}

// ====== Base score: M+B + V3 5-layer traffic ======
static inline float base_score(int a_loc, int tid, int step) {
    (void)step;
    auto it = g_env->task_pool.find(tid);
    if (it == g_env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;
    auto& locs = it->second.locations;
    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;
    int total = 0, cm = a_m, cb = a_b;
    for (int i = 0; i < (int)locs.size(); i++) {
        int tm, tb;
        Centerline::project(locs[i], tm, tb);
        if (tm < 0) { total += 1000; continue; }
        total += std::abs(cm - tm) + std::abs(cb - tb);
        cm = tm; cb = tb;
    }
    float mb = 1.0f / (float)(total + 1);
    
    auto cit = Centerline::g_task_zone_cache.find(tid);
    if (cit == Centerline::g_task_zone_cache.end() || Centerline::g_zone24_density.empty())
        return mb;
    auto& info = cit->second;
    
    // Layer 1: Cold bonus
    float cold = 0;
    if (info.task_zone >= 0 && info.task_zone < (int)Centerline::g_zone24_density.size()) {
        int d = Centerline::g_zone24_density[info.task_zone];
        if (d < COLD_THRESHOLD)
            cold = (float)(COLD_THRESHOLD - d) * COLD_WEIGHT;
    }
    
    // Layer 2: Hot penalty (task in hot zone)
    float hot = 0;
    if (info.task_zone >= 0 && info.task_zone < (int)Centerline::g_zone24_density.size()) {
        int d = Centerline::g_zone24_density[info.task_zone];
        if (d >= HOT_THRESHOLD)
            hot = (float)(d - HOT_THRESHOLD) * HOT_WEIGHT;
    }
    
    // Layer 3: Route penalty (path through hot zones)
    float route = 0;
    for (int z : info.route_zones) {
        if (z >= 0 && z < (int)Centerline::g_zone24_density.size()) {
            int d = Centerline::g_zone24_density[z];
            if (d >= HOT_THRESHOLD)
                route += (float)(d - HOT_THRESHOLD) * ROUTE_WEIGHT;
        }
    }
    
    // Layer 4: Direction bonus/penalty
    float dir_b = 0, dir_p = 0;
    if (a_m >= 0) {
        int intent = (info.m_start > a_m) ? +1 : -1;
        int rs = std::max(0, std::min(a_m, info.m_start));
        int re = std::min(Centerline::g_total_len, std::max(a_m, info.m_start));
        std::unordered_set<int> path_zones;
        for (int m = rs; m <= re; m++) {
            if (m < (int)g_zone_at_main.size()) {
                int z = g_zone_at_main[m];
                if (z >= 0) path_zones.insert(z);
            }
        }
        for (int z : path_zones) {
            if (z >= 0 && z < (int)Centerline::g_zone_direction.size()) {
                int f = Centerline::g_zone_direction[z];
                if (f == 0) continue;
                if (f == intent) dir_b += DIR_MATCH_WEIGHT;
                else dir_p += CROSS_WEIGHT;
            }
        }
    }
    
    return mb + cold - hot - route + dir_b - dir_p;
}


// ====== Pre-tick updates ======
void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_zone_direction(env);
    
    // Cache zone info for new tasks
    for (int tid : env->new_tasks) {
        if (Centerline::g_task_zone_cache.count(tid)) continue;
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        
        Centerline::TaskZoneInfo info;
        const auto& locs = it->second.locations;
        int min_m = INT_MAX/2, max_m = 0;
        for (size_t ei = 0; ei < locs.size(); ei++) {
            int m, b;
            Centerline::project(locs[ei], m, b);
            if (m >= 0) { if (m < min_m) min_m = m; if (m > max_m) max_m = m; }
        }
        info.m_start = min_m; info.m_end = max_m;
        
        std::unordered_set<int> seen;
        int m_lo = std::max(0, min_m - 24);
        int m_hi = std::min(Centerline::g_total_len, max_m + 24);
        for (int m = m_lo; m <= m_hi; m++) {
            if (m < (int)g_zone_at_main.size()) {
                int z = g_zone_at_main[m];
                if (z >= 0 && !seen.count(z)) {
                    seen.insert(z);
                    info.route_zones.push_back(z);
                }
            }
        }
        
        int tm, tb;
        Centerline::project(locs[0], tm, tb);
        info.task_zone = (tm >= 0 && tm < (int)g_zone_at_main.size()) ? g_zone_at_main[tm] : -1;
        Centerline::g_task_zone_cache[tid] = info;
    }
}

// ====== Plan: greedy + V3 traffic scoring ======
void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    pre_tick_updates(env);
    
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;
    
    // ---- Collect free agents + tasks ----
    std::vector<int> free_agents;
    std::unordered_set<int> free_tasks;
    
    for (int a = 0; a < env->num_of_agents; a++) {
        if (env->curr_task_schedule[a] < 0) free_agents.push_back(a);
    }
    
    std::unordered_set<int> assigned_tasks;
    for (int a = 0; a < env->num_of_agents; a++) {
        int cur = env->curr_task_schedule[a];
        if (cur >= 0) assigned_tasks.insert(cur);
    }
    
    for (int tid : env->new_tasks) free_tasks.insert(tid);
    // All tasks not assigned + still alive → free
    for (auto& kv : env->task_pool) {
        int tid = kv.first;
        if (!assigned_tasks.count(tid))
            free_tasks.insert(tid);
    }
    
    // Phase 1: Continue multi-errand tasks
    for (auto it = free_agents.begin(); it != free_agents.end(); ) {
        int a = *it;
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) &&
            env->task_pool[cur].locations.size() > 1) {
            proposed_schedule[a] = cur;
            free_tasks.erase(cur);
            it = free_agents.erase(it);
        } else {
            ++it;
        }
    }
    
    if (free_agents.empty()) return;
    
    // ---- Phase 2: Greedy assignment ----
    if (!free_agents.empty() && !free_tasks.empty() && std::chrono::steady_clock::now() <= endtime) {
        std::unordered_map<int, std::vector<int>> tasks_by_zone;
        for (int tid : free_tasks) {
            if (env->task_pool.count(tid) && !env->task_pool[tid].locations.empty())
                tasks_by_zone[zid(env->task_pool[tid].locations[0], cols)].push_back(tid);
        }
        
        std::unordered_set<int> done_tasks;
        for (int a : free_agents) {

            if (proposed_schedule[a] >= 0) continue;
            
            int a_loc = env->curr_states[a].location;
            int a_zone = zid(a_loc, cols);
            int best_tid = -1;
            float best_score = -1e9f;
            
            // Same zone
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end()) {
                for (int tid : sz->second) {
                    if (done_tasks.count(tid)) continue;
                    float bs = base_score(a_loc, tid, step);
                    if (bs > best_score) { best_score = bs; best_tid = tid; }
                }
            }
            
            // Adjacent zones (up to 20 each)
            if (best_tid < 0) {
                for (auto& kv : tasks_by_zone) {
                    int z2 = kv.first;
                    if (z2 == a_zone || zh_dist(a_zone, z2) > 1) continue;
                    int checked = 0;
                    for (int tid : kv.second) {
                        if (done_tasks.count(tid)) continue;
                        if (checked++ >= 20) break;
                        float bs = base_score(a_loc, tid, step);
                        if (bs > best_score) { best_score = bs; best_tid = tid; }
                    }
                }
            }
            
            if (best_tid >= 0) {
                proposed_schedule[a] = best_tid;
                done_tasks.insert(best_tid);
            } else {
                proposed_schedule[a] = -1;
            }
        }
    }
    
    // Remaining unmatched → -1
    for (int a : free_agents) {
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
    }
}

} // namespace strategy_centerline
