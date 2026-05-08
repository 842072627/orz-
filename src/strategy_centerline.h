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

// ====== Config ======

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
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d\n",
           env->num_of_agents, env->cols, env->rows);
}

// ====== Base score: pure centerline M+B makespan ======
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
    return 1.0f / (float)(total + 1);
}

// ====== Plan: pure greedy (no Hungarian, no bonuses) ======
void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
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
