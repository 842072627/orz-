// timetable.h - Parallel timetable prediction module
// Runs alongside direction lock. Does NOT affect scheduling decisions (yet).
// Logs: direction lock vs timetable decision comparison for offline analysis.
#pragma once
#include "SharedEnv.h"
#include "config.h"
#include "centerline.h"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <vector>

namespace Timetable {

static constexpr int LOG_INTERVAL = 100;   // log every 100 ticks
static constexpr float WEIGHT_PENDING = 1.0f;  // each pending agent counts as 1.0 toward threshold

// ====== State ======
// Capacity is fixed at ZONE_HARD_CAP (15), same as direction lock threshold
static std::vector<int> g_mp_capacity;      // [si] = capacity (fixed at ZONE_HARD_CAP)
static std::vector<int> g_mp_pending;        // [si] = agents heading toward this MP
static std::vector<float> g_mp_total_load;   // [si] = density + pending * weight
static int g_nmp = 0;
static int g_log_fd = -1;
static int g_last_log_tick = -1;
static int g_tick_since_init = 0;

// Decision counters
static int g_total_decisions = 0;
static int g_agree_decisions = 0;    // direction_lock == timetable
static int g_dl_block_tt_pass = 0;   // dir lock blocked, timetable said OK
static int g_dl_pass_tt_block = 0;   // dir lock allowed, timetable said block

// ====== Initialize ======
// Capacity = ZONE_HARD_CAP (15), same as direction lock threshold
inline void init() {
    g_nmp = (int)Centerline::g_mp_count;
    if (g_nmp <= 0) return;
    g_mp_capacity.assign(g_nmp, ZONE_HARD_CAP);
    g_mp_pending.assign(g_nmp, 0);
    g_mp_total_load.assign(g_nmp, 0.0f);
    g_log_fd = -1;
    g_last_log_tick = -1;
    g_tick_since_init = 0;
    g_total_decisions = 0;
    g_agree_decisions = 0;
    g_dl_block_tt_pass = 0;
    g_dl_pass_tt_block = 0;
}

// ====== No capacity adaptation needed - using fixed ZONE_HARD_CAP ======
// Capacity is same as direction lock threshold (15)
// Timetable predicts: will density + pending exceed this threshold?
inline void update_capacity() {
    // No-op: capacity is fixed at ZONE_HARD_CAP
}

// ====== Recompute pending counts ======
// For each agent with a task, add 1 to pending for each MP on its path
inline void update_pending(SharedEnvironment* env) {
    if (g_nmp <= 0) return;
    std::fill(g_mp_pending.begin(), g_mp_pending.end(), 0);
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int task_id = env->curr_task_schedule[a];
        if (task_id < 0) continue;  // no task assigned
        
        auto it = env->task_pool.find(task_id);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        
        // Project agent location to main_dist
        int m_curr = -1, b_curr;
        Centerline::project(a_loc, m_curr, b_curr);
        if (m_curr < 0) continue;
        
        // Project task errands
        auto& locs = it->second.locations;
        int prev_m = m_curr;
        
        for (int loc : locs) {
            int tm, tb;
            Centerline::project(loc, tm, tb);
            if (tm < 0) continue;
            
            int lo = std::min(prev_m, tm);
            int hi = std::max(prev_m, tm);
            
            // For each main point in range [lo, hi], increment pending
            for (int si = 0; si < g_nmp && si < (int)Centerline::g_mp_main_dist.size(); si++) {
                int si_m = Centerline::g_mp_main_dist[si];
                if (si_m >= lo && si_m <= hi) {
                    g_mp_pending[si]++;
                }
            }
            prev_m = tm;
        }
    }
    
    // Update total load
    for (int si = 0; si < g_nmp; si++) {
        float density = (si < (int)Centerline::g_mp_density.size()) ? (float)Centerline::g_mp_density[si] : 0.0f;
        g_mp_total_load[si] = density + g_mp_pending[si] * WEIGHT_PENDING;
    }
}

// ====== Check: does timetable approve this task for this agent? ======
// Returns: 1 = pass (timetable OK), 0 = block, -1 = agent not on main line
inline int check_assignment(int agent_loc, const std::vector<int>& task_locs) {
    if (g_nmp <= 0) return 1;
    
    int m_curr = -1, b_curr;
    Centerline::project(agent_loc, m_curr, b_curr);
    if (m_curr < 0) return -1;
    
    int prev_m = m_curr;
    
    for (int loc : task_locs) {
        int tm, tb;
        Centerline::project(loc, tm, tb);
        if (tm < 0) continue;
        
        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        
        for (int si = 0; si < g_nmp && si < (int)Centerline::g_mp_main_dist.size(); si++) {
            int si_m = Centerline::g_mp_main_dist[si];
            if (si_m < lo || si_m > hi) continue;
            
            float load = g_mp_total_load[si];
            int cap = g_mp_capacity[si];
            
            // After this assignment, load would increase by WEIGHT_PENDING
            if (load + WEIGHT_PENDING > cap) {
                return 0;  // timetable says block
            }
        }
        prev_m = tm;
    }
    return 1;  // timetable says pass
}

// ====== Log timetable state ======
inline void log_state(SharedEnvironment* env) {
    int tick = env->curr_timestep;
    if (tick % LOG_INTERVAL != 0) return;
    if (tick == g_last_log_tick) return;
    g_last_log_tick = tick;
    
    // Open log file on first write
    if (g_log_fd < 0) {
        const char* path = "C:/Users/Administrator/.openclaw/workspace/timetable_log.csv";
        FILE* f = fopen(path, "w");
        if (f) {
            fprintf(f, "tick,si,main_d,density,pending,total_load,capacity,lock_dir,"
                       "tt_assign_check,dl_auto\n");
            fclose(f);
        }
        g_log_fd = 1;  // non-negative means initialized
    }
    
    FILE* f = fopen("C:/Users/Administrator/.openclaw/workspace/timetable_log.csv", "a");
    if (!f) return;
    
    for (int si = 0; si < g_nmp; si++) {
        int density = (si < (int)Centerline::g_mp_density.size()) ? Centerline::g_mp_density[si] : 0;
        int lock_dir = (si < (int)Centerline::g_mp_lock_dir.size()) ? Centerline::g_mp_lock_dir[si] : 0;
        int main_d = (si < (int)Centerline::g_mp_main_dist.size()) ? Centerline::g_mp_main_dist[si] : -1;
        int cap = (si < (int)g_mp_capacity.size()) ? g_mp_capacity[si] : 15;
        float load = (si < (int)g_mp_total_load.size()) ? g_mp_total_load[si] : 0.0f;
        int pending = (si < (int)g_mp_pending.size()) ? g_mp_pending[si] : 0;
        
        // DL auto-block: would direction lock automatically block this MP?
        int dl_auto = 0;
        if (density >= 15) dl_auto = 1;
        
        fprintf(f, "%d,%d,%d,%d,%d,%.1f,%d,%d,0,%d\n",
                tick, si, main_d, density, pending, load, cap, lock_dir,
                dl_auto);
    }
    fclose(f);
}

// ====== Log a single decision comparison ======
inline void log_decision(int tick, int si, int dl_block, int tt_block) {
    g_total_decisions++;
    
    if (dl_block == tt_block) {
        g_agree_decisions++;
    } else if (dl_block && !tt_block) {
        g_dl_block_tt_pass++;
    } else if (!dl_block && tt_block) {
        g_dl_pass_tt_block++;
    }
    
    // Log detailed comparison every 1000 decisions
    if (g_total_decisions % 1000 == 0) {
        FILE* f = fopen("C:/Users/Administrator/.openclaw/workspace/timetable_log.csv", "a");
        if (f) {
            float agree_rate = 100.0f * g_agree_decisions / g_total_decisions;
            fprintf(f, "#DECISION_SUMMARY,%d,%d,%.1f,%d,%d,%d\n",
                    tick, g_total_decisions, agree_rate,
                    g_agree_decisions, g_dl_block_tt_pass, g_dl_pass_tt_block);
            fclose(f);
        }
    }
}

// ====== Close out log with final summary ======
inline void close_log(int tick) {
    FILE* f = fopen("C:/Users/Administrator/.openclaw/workspace/timetable_log.csv", "a");
    if (f && g_total_decisions > 0) {
        float agree_rate = 100.0f * g_agree_decisions / g_total_decisions;
        fprintf(f, "#FINAL,%d,%d,%.1f,%d,%d,%d\n",
                tick, g_total_decisions, agree_rate,
                g_agree_decisions, g_dl_block_tt_pass, g_dl_pass_tt_block);
        fprintf(f, "#CAPACITIES");
        for (int si = 0; si < g_nmp; si++) {
            fprintf(f, ",%d", g_mp_capacity[si]);
        }
        fprintf(f, "\n");
        fclose(f);
    }
}

} // namespace Timetable
