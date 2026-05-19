// garbage_pile.h - Agent and task garbage pile
// Idles agents when the main line is congested.
// Does NOT affect direction lock. Works alongside timetable.
#pragma once
#include "SharedEnv.h"
#include "config.h"
#include "centerline.h"
#include "timetable.h"
#include "pibt_lock.h"
#include <algorithm>
#include <cstdio>
#include <vector>

namespace GarbagePile {

static constexpr float GARBAGE_LOAD_THRESHOLD = 0.9f;  // idle agent if MP load > 90% capacity
static constexpr float GP_TASK_GARBAGE_RATIO = 0.15f;     // defer bottom 15% tasks by timetable score
static constexpr int LOG_INTERVAL = 100;
static int g_log_fd = -1;
static int g_last_log_tick = -1;

// ====== Agent garbage pile ======
// After scheduling, idle agents that are in congested areas
inline void update_agent_garbage(SharedEnvironment* env,
                                  const std::vector<int>& free_agents,
                                  std::vector<int>& proposed_schedule) {
    if (!USE_IDLE_CAP) return;  // only active when idle cap is on
    
    for (int a : free_agents) {
        if (proposed_schedule[a] >= 0) continue;  // already got a task
        
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        
        int m = -1, b;
        Centerline::project(a_loc, m, b);
        if (m < 0) continue;
        
        // Find nearest main point
        int nearest = 0;
        int best_d = 999999;
        for (int si = 0; si < (int)Centerline::g_mp_count && si < (int)Centerline::g_mp_main_dist.size(); si++) {
            int si_m = Centerline::g_mp_main_dist[si];
            int d = std::abs(m - si_m);
            if (d < best_d) { best_d = d; nearest = si; }
        }
        
        // Check total load vs capacity
        float load = Timetable::g_mp_total_load[nearest];
        int cap = Timetable::g_mp_capacity[nearest];
        
        if (cap > 0 && load >= cap * GARBAGE_LOAD_THRESHOLD) {
            // Agent is in a congested area → garbage pile (idle)
            if (a < PIBT_Lock::MAX_AGENTS) {
                PIBT_Lock::g_agent_blocked[a] = 1;
            }
            proposed_schedule[a] = -1;
        }
    }
}

// ====== Task garbage pile ======
// Assign a "garbage score" to each task; the bottom X% are deferred
// The score is based on timetable load along the task's path
inline float task_garbage_score(int agent_loc, int task_id, SharedEnvironment* env) {
    auto it = env->task_pool.find(task_id);
    if (it == env->task_pool.end() || it->second.locations.empty()) return -1e9f;
    
    int m_curr = -1, b_curr;
    Centerline::project(agent_loc, m_curr, b_curr);
    if (m_curr < 0) return 1.0f;
    
    float max_load = 0.0f;
    int prev_m = m_curr;
    
    for (int loc : it->second.locations) {
        int tm, tb;
        Centerline::project(loc, tm, tb);
        if (tm < 0) continue;
        
        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        
        for (int si = 0; si < Timetable::g_nmp && si < (int)Centerline::g_mp_main_dist.size(); si++) {
            int si_m = Centerline::g_mp_main_dist[si];
            if (si_m < lo || si_m > hi) continue;
            
            float load = Timetable::g_mp_total_load[si];
            int cap = Timetable::g_mp_capacity[si];
            
            float load_ratio = (cap > 0) ? load / cap : 1.0f;
            if (load_ratio > max_load) max_load = load_ratio;
        }
        prev_m = tm;
    }
    
    // Return "garbage score" - lower is worse (more likely to be deferred)
    return 1.0f - max_load;
}

// ====== Log garbage pile state ======
inline void log_state(SharedEnvironment* env, int garbage_count) {
    int tick = env->curr_timestep;
    if (tick % LOG_INTERVAL != 0) return;
    if (tick == g_last_log_tick) return;
    g_last_log_tick = tick;
    
    if (g_log_fd < 0) {
        const char* path = "C:/Users/Administrator/.openclaw/workspace/garbage_log.csv";
        FILE* f = fopen(path, "w");
        if (f) {
            fprintf(f, "tick,garbage_agents,total_agents,ratio\n");
            fclose(f);
        }
        g_log_fd = 1;
    }
    
    FILE* f = fopen("C:/Users/Administrator/.openclaw/workspace/garbage_log.csv", "a");
    if (!f) return;
    
    int total_free = 0;
    for (int a = 0; a < env->num_of_agents; a++) {
        if (env->curr_task_schedule[a] < 0) total_free++;
    }
    
    fprintf(f, "%d,%d,%d,%.3f\n", tick, garbage_count, total_free,
            total_free > 0 ? (float)garbage_count / total_free : 0.0f);
    fclose(f);
}

} // namespace GarbagePile
