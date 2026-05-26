// strategy_centerline.h — V5: Dual-lane system. No direction locks, no prediction checks.
// Lane discipline enforced by PIBT (centerline.h g_cell_lane). Scheduler only does distance-based task assignment.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include "pibt_lock.h"
#include "idle_cap.h"
#include "corridor_log.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <vector>
#include <chrono>

namespace strategy_centerline {

static SharedEnvironment* g_env = nullptr;
static std::vector<int> g_parked_task;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
static inline int unified_dist(int loc_a, int loc_b) {
    int d = BFSTable::get_dist(loc_a, loc_b);
    if (d > 0) return d;
    int a_m, a_b, b_m, b_b;
    Centerline::project(loc_a, a_m, a_b);
    Centerline::project(loc_b, b_m, b_b);
    if (a_m >= 0 && b_m >= 0) return std::abs(b_m - a_m) + std::abs(b_b - a_b);
    return std::abs(loc_a / g_env->cols - loc_b / g_env->cols) + std::abs(loc_a % g_env->cols - loc_b % g_env->cols);
}

void init(SharedEnvironment* env) {
    g_env = env;
    Centerline::init(env);
#if USE_DIAG_LOG
    printf("DUAL_LANE: agents=%d, map=%dx%d\n", env->num_of_agents, env->cols, env->rows);
#endif
}

// base_score: distance-based + lane check
static inline float base_score(int a_loc, int tid, SharedEnvironment* env) {
    auto it = env->task_pool.find(tid);
    if (it == env->task_pool.end() || it->second.locations.empty()) return -1e9f;
    auto& locs = it->second.locations;
    
    // Lane check temporarily disabled — PIBT handles lane preference
    
    // Distance from agent to task1
    float d = (float)unified_dist(a_loc, locs[0]);
    float task1_dist = d;
    
    if (locs.size() >= 2) {
        float d2 = (float)unified_dist(locs[0], locs[1]);
        if (d2 < 1) d2 = 1;
        return -(d * 0.01f + d2);
    }
    return -(task1_dist + 1);
}

void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_mp_density(env);
    
    CenterlineAccess::g_branch_dist   = &Centerline::g_branch_dist;
    CenterlineAccess::g_main_dist     = &Centerline::g_main_dist;
    CenterlineAccess::g_zone_is_main  = &Centerline::g_zone_is_main;
    CenterlineAccess::g_mp_loc        = &Centerline::g_mp_loc;
    CenterlineAccess::g_mp_main_dist  = &Centerline::g_mp_main_dist;
    CenterlineAccess::g_mp_count      = Centerline::g_mp_count;
    CenterlineAccess::g_n24c          = Centerline::g_n24c;
    CenterlineAccess::g_n24r          = Centerline::g_n24r;
    CenterlineAccess::g_cell_lane     = &Centerline::g_cell_lane;
}

void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    pre_tick_updates(env);
    
    if ((int)g_parked_task.size() != env->num_of_agents)
        g_parked_task.assign(env->num_of_agents, -1);
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_parked_task[a] = g_parked_task[a];
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_agent_blocked[a] = 0;

    auto t0 = std::chrono::steady_clock::now();
    TimePoint endtime = t0 + std::chrono::milliseconds(limit);
    int cols = env->cols;

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
        int tid = kv.first;
        if (!assigned_tasks.count(tid)) free_tasks.insert(tid);
    }

    // Phase 1: Continue multi-errand tasks
    for (auto it = free_agents.begin(); it != free_agents.end(); ) {
        int a = *it;
        int parked = (a < (int)g_parked_task.size()) ? g_parked_task[a] : -1;
        if (parked >= 0) {
            int a_loc = env->curr_states[a].location;
            float bs = base_score(a_loc, parked, env);
            if (bs > 0) {
                proposed_schedule[a] = parked;
                g_parked_task[a] = -1;
                free_tasks.erase(parked);
                it = free_agents.erase(it);
            } else {
                proposed_schedule[a] = -1;
                ++it;
            }
            continue;
        }
        int cur_t = env->curr_task_schedule[a];
        if (cur_t >= 0 && env->task_pool.count(cur_t) &&
            env->task_pool[cur_t].locations.size() > 1) {
            proposed_schedule[a] = cur_t;
            free_tasks.erase(cur_t);
            it = free_agents.erase(it);
        } else {
            ++it;
        }
    }

    if (free_agents.empty()) {
        for (int a = 0; a < env->num_of_agents; a++)
            if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
        return;
    }

    // Phase 2: Distance-based greedy assignment
    struct TaskCand { int tid; float task_dist; float agent_dist; };
    std::unordered_set<int> done_tasks;

    for (int a : free_agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (proposed_schedule[a] >= 0) continue;
        int a_loc = env->curr_states[a].location;

        std::vector<TaskCand> candidates;
        for (int tid : free_tasks) {
            if (done_tasks.count(tid)) continue;
            auto it = env->task_pool.find(tid);
            if (it == env->task_pool.end() || it->second.locations.size() < 1) continue;
            
            float agent_dist = (float)unified_dist(a_loc, it->second.locations[0]);
            float task_dist = 0;
            if (it->second.locations.size() >= 2)
                task_dist = (float)unified_dist(it->second.locations[0], it->second.locations[1]);
            candidates.push_back({tid, task_dist, agent_dist});
        }

        // Sort: primary by task1→task2 distance, secondary by agent→task1
        std::sort(candidates.begin(), candidates.end(),
            [](const TaskCand& a, const TaskCand& b) {
                if (a.task_dist != b.task_dist) return a.task_dist < b.task_dist;
                return a.agent_dist < b.agent_dist;
            });

        int best_tid = -1;
        float best_score = -1e9f;
        for (auto& cand : candidates) {
            float bs = base_score(a_loc, cand.tid, env);
            if (bs > best_score) { best_score = bs; best_tid = cand.tid; }
        }

        if (best_tid >= 0) {
            proposed_schedule[a] = best_tid;
            auto tit = env->task_pool.find(best_tid);
            g_parked_task[a] = (tit != env->task_pool.end() && tit->second.locations.size() > 2) ? best_tid : -1;
            done_tasks.insert(best_tid);
        } else {
            proposed_schedule[a] = -1;
        }
    }

    for (int a : free_agents)
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
}

} // namespace strategy_centerline
