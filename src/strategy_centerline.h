// strategy_centerline.h — Centerline M+B makespan greedy scheduler
// Uses Centerline::project() for distance along corridor.
// Init done in preprocess phase. Pure greedy, no congestion/bonuses.

#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include "pibt_lock.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cstdint>
#include <chrono>
#include <climits>
#include <cmath>
#include <bitset>

namespace strategy_centerline {

// ====== Traffic constants ======
static constexpr float DIR_MATCH_WEIGHT = 0.003f;
static constexpr float CROSS_WEIGHT = 0.010f;

// ====== Route penalty constants ======
static constexpr int ROUTE_THRESHOLD = 20;
// Divisor table: d=20 -> /1.0, d=40 -> /3.0
static constexpr float DIVISOR_TABLE[21] = {
    1.0f, 1.1f, 1.2f, 1.3f, 1.4f,
    1.5f, 1.6f, 1.7f, 1.8f, 1.9f,
    2.0f, 2.1f, 2.2f, 2.3f, 2.4f,
    2.5f, 2.6f, 2.7f, 2.8f, 2.9f,
    3.0f
};

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
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d, segments=%zu\n",
           env->num_of_agents, env->cols, env->rows, Centerline::g_segments.size());
}

// ====== Base score: M+B + 全程穿越惩罚 + 方向微操 ======
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

    // M+B makespan
    int total = 0, cm = a_m, cb = a_b;
    for (int i = 0; i < (int)locs.size(); i++) {
        int tm, tb;
        Centerline::project(locs[i], tm, tb);
        if (tm < 0) { total += 1000; continue; }
        total += std::abs(cm - tm) + std::abs(cb - tb);
        cm = tm; cb = tb;
    }
    float mb = 1.0f / (float)(total + 1);

    // Task waypoint main range
    int task_min_m = INT_MAX/2, task_max_m = 0;
    for (auto& loc : locs) {
        int m, b;
        Centerline::project(loc, m, b);
        if (m >= 0) {
            if (m < task_min_m) task_min_m = m;
            if (m > task_max_m) task_max_m = m;
        }
    }
    if (task_min_m >= INT_MAX/2) return mb;

    // 全程扫描：穿越惩罚 + 方向微操
    float dir_bonus = 0, dir_cross = 0;
    int d_max = 0;
    int intent = (task_min_m > a_m) ? +1 : -1;
    int rs = std::max(0, std::min(a_m, task_min_m - 24));
    int re = std::min(Centerline::g_total_len, std::max(a_m, task_max_m + 24));

    std::bitset<256> visited;
    for (int m = rs; m <= re; m++) {
        int si = Centerline::m_to_segment(m);
        if (si < 0 || si >= (int)Centerline::g_segments.size() || visited[si])
            continue;
        visited[si] = true;

        int zid = Centerline::g_segments[si].zone_id;
        if (zid < 0 || zid >= (int)Centerline::g_zone24_density.size())
            continue;

        int d = Centerline::g_zone24_density[zid];
        if (d > d_max) d_max = d;

        // 方向硬约束：热 segment 只允许顺流（纯 1D）
        int f = Centerline::g_seg_lock_dir[si];
        if (f != 0 && f != intent)
            return -1e9f;

        // 方向微操
        if (f != 0) {
            if (f == intent) dir_bonus += DIR_MATCH_WEIGHT;
            else             dir_cross += CROSS_WEIGHT;
        }
    }

    // 除数：d=20→1.0, d=40→3.0
    float divisor = 1.0f;
    if (d_max > ROUTE_THRESHOLD) {
        int idx = (d_max > 40) ? 40 : d_max;
        divisor = DIVISOR_TABLE[idx - ROUTE_THRESHOLD];
    }

    return (mb + dir_bonus - dir_cross) / divisor;
}


// ====== Pre-tick updates ======
void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_zone_direction(env);
}

// ====== Plan: greedy + V3 traffic scoring ======
void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    pre_tick_updates(env);

    // Set agent direction block flags for PIBT (2D line based)
    int n24c = (env->cols + 23) / 24;
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++) {
        PIBT_Lock::g_agent_blocked[a] = 0;
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;
        auto tit = env->task_pool.find(tid);
        if (tit == env->task_pool.end() || tit->second.locations.empty()) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;

        int a_r = a_loc / env->cols, a_c = a_loc % env->cols;
        int a_zone = (a_r / 24) * n24c + (a_c / 24);
        if (a_zone >= 0 && a_zone < (int)Centerline::g_zone24_density.size() &&
            Centerline::g_zone24_density[a_zone] >= 28)
            continue;  // already in hot zone, don't block

        int t_loc = tit->second.locations[0];
        int t_r = t_loc / env->cols, t_c = t_loc % env->cols;

        int dx = t_c - a_c, dy = t_r - a_r;
        int max_steps = std::min(24, std::max(std::abs(dx), std::abs(dy)));
        if (max_steps < 1) continue;

        // Agent intent: along main_dist toward task
        int a_m, a_b, t_m, t_b;
        Centerline::project(a_loc, a_m, a_b);
        Centerline::project(t_loc, t_m, t_b);
        int intent = (t_m > a_m) ? +1 : -1;

        bool blocked = false;
        for (int s = 1; s <= max_steps; s++) {
            int px = a_c + (dx * s) / max_steps;
            int py = a_r + (dy * s) / max_steps;
            int ploc = py * env->cols + px;
            int zone = (py / 24) * n24c + (px / 24);
            if (zone >= 0 && zone < (int)Centerline::g_zone24_density.size() &&
                Centerline::g_zone24_density[zone] >= 28) {
                // Check direction: only block if segment lock opposes intent
                int pm, pb;
                Centerline::project(ploc, pm, pb);
                if (pm >= 0) {
                    int si = Centerline::m_to_segment(pm);
                    if (si >= 0 && si < (int)Centerline::g_seg_lock_dir.size() &&
                        Centerline::g_seg_lock_dir[si] != 0 &&
                        Centerline::g_seg_lock_dir[si] != intent) {
                        blocked = true;
                        break;
                    }
                }
            }
        }

        PIBT_Lock::g_agent_blocked[a] = blocked ? 1 : 0;
    }
    
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
