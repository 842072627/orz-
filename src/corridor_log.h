// corridor_log.h - Per-tick corridor density stats
// Task completion analysis reads from JSON events at analysis time.
#pragma once
#include "SharedEnv.h"
#include "centerline.h"
#include <cstdio>
#include <algorithm>
#include <vector>

namespace CorridorLog {

static FILE* g_fd = nullptr;
static FILE* g_agent_fd = nullptr;
static FILE* g_mp_fd = nullptr;
static FILE* g_assign_fd = nullptr;
static int g_call_count = 0;
static SharedEnvironment* g_env_ptr = nullptr;

inline void init() {
#if USE_DIAG_LOG
    g_fd = fopen("C:/Users/Administrator/.openclaw/workspace/corridor_log.csv", "w");
    if (g_fd) {
        fprintf(g_fd, "tick,on_mainline,free_agents,avg_dens,"
                      "max_dens,p90_dens,overload15_pct,"
                      "n_mp_active,n_locked,n_hot_dens,n_flow_unlock,n_proactive\n");
        fflush(g_fd);
    }
    g_agent_fd = fopen("C:/Users/Administrator/.openclaw/workspace/agent_log.csv", "w");
    if (g_agent_fd) {
        fprintf(g_agent_fd, "tick,aid,x,y,m,b,cell_width,speed,task_id,errand_idx,lock_dir\n");
        fflush(g_agent_fd);
    }
    g_mp_fd = fopen("C:/Users/Administrator/.openclaw/workspace/mp_log.csv", "w");
    if (g_mp_fd) {
        fprintf(g_mp_fd, "tick,si,main_m,width,lock_dir,future_right,future_left,density\n");
        fflush(g_mp_fd);
    }
    g_assign_fd = fopen("C:/Users/Administrator/.openclaw/workspace/assign_log.csv", "w");
    if (g_assign_fd) {
        fprintf(g_assign_fd, "tick,exempt,branch,dirlock,cap,accepted\n");
        fflush(g_assign_fd);
    }
#endif
}

inline void set_env(SharedEnvironment* env) {
    g_env_ptr = env;
}

inline void log_tick(SharedEnvironment* env) {
    if (!g_fd) return;
    g_call_count++;
    int approx_tick = g_call_count * 10;
    int n_agents = env->num_of_agents;
    
    int on_main = 0;
    int free_ag = 0;
    for (int a = 0; a < n_agents && a < 10000; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int m = -1, b = -1;
        Centerline::project(loc, m, b);
        if (m >= 0 && b >= 0 && b < 8) on_main++;
        if (env->curr_task_schedule[a] < 0) free_ag++;
    }
    
    int n_mp = std::min(220, (int)Centerline::g_mp_density.size());
    long long sum_dens = 0;
    int max_dens = 0;
    int overload15 = 0;
    std::vector<int> dens_vals;
    dens_vals.reserve(n_mp);
    
    for (int si = 0; si < n_mp; si++) {
        int d = Centerline::g_mp_density[si];
        sum_dens += d;
        if (d > max_dens) max_dens = d;
        if (d >= 15) overload15++;
        if (d > 0) dens_vals.push_back(d);
    }
    
    float avg = n_mp > 0 ? (float)sum_dens / n_mp : 0;
    float overload_pct = n_mp > 0 ? 100.0f * overload15 / n_mp : 0;
    
    std::sort(dens_vals.begin(), dens_vals.end());
    float p90 = 0;
    if (!dens_vals.empty()) {
        int idx = (int)(dens_vals.size() * 0.9);
        if (idx >= (int)dens_vals.size()) idx = (int)dens_vals.size() - 1;
        p90 = (float)dens_vals[idx];
    }
    
    // Direction lock stats
    int n_locked = 0;
    for (int si = 0; si < n_mp; si++) {
        if (Centerline::g_mp_lock_dir[si] != 0) n_locked++;
    }
    
    fprintf(g_fd, "%d,%d,%d,%.1f,%d,%.0f,%.1f,%d,%d,%d,%d,%d\n",
            approx_tick, on_main, free_ag, avg, max_dens,
            p90, overload_pct, (int)dens_vals.size(),
            n_locked);
    // Columns: tick,mainline,free,avg,max,p90,overload%,n_active,n_locked,n_hot_dens,n_flow_unlock,n_proactive
    fflush(g_fd);
    
    // Per-agent log (sampled: every 3rd tick, only agents with tasks)
    if (g_agent_fd && (g_call_count % 3 == 1)) {
        static std::vector<int> prev_main;
        if ((int)prev_main.size() < env->num_of_agents)
            prev_main.assign(env->num_of_agents, -1);
        for (int a = 0; a < env->num_of_agents && a < 5000; a++) {
            int tid = env->curr_task_schedule[a];
            if (tid < 0) { prev_main[a] = -1; continue; } // idle, skip
            int loc = env->curr_states[a].location;
            if (loc < 0) continue;
            int m = -1, b = -1;
            Centerline::project(loc, m, b);
            if (m < 0) { prev_main[a] = -1; continue; }
            int cw = (int)Centerline::g_cell_width[loc];
            if (cw > 50000) continue; // wall
            int speed = (prev_main[a] >= 0) ? std::abs(m - prev_main[a]) : 0;
            prev_main[a] = m;
            // Find nearest MP's lock_dir
            int ld = 0;
            int best_si = 0, best_d = 999999;
            for (int si = 0; si < Centerline::g_mp_count; si++) {
                int mloc = Centerline::g_mp_loc[si];
                if (mloc < 0) continue;
                int dr = std::abs(loc / env->cols - mloc / env->cols);
                int dc = std::abs(loc % env->cols - mloc % env->cols);
                int d = dr + dc;
                if (d < best_d) { best_d = d; best_si = si; }
            }
            ld = Centerline::g_mp_lock_dir[best_si];
            int errand_idx = 0; // not tracked in detail here
            int ax = loc % env->cols;
            int ay = loc / env->cols;
            fprintf(g_agent_fd, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    approx_tick, a, ax, ay, m, b, cw, speed, tid, errand_idx, ld);
        }
        fflush(g_agent_fd);
    }
    
    // Main point log (sampled: every 3rd tick)
    if (g_mp_fd && (g_call_count % 3 == 1)) {
        for (int si = 0; si < Centerline::g_mp_count; si++) {
            int mloc = Centerline::g_mp_loc[si];
            if (mloc < 0) continue;
            int mm = Centerline::g_mp_main_dist[si];
            int w = (int)Centerline::g_cell_width[mloc];
            if (w > 50000) continue;
            fprintf(g_mp_fd, "%d,%d,%d,%d,%d,%.1f,%.1f,%d\n",
                    approx_tick, si, mm, w,
                    Centerline::g_mp_lock_dir[si],
                    Centerline::g_mp_future_right[si],
                    Centerline::g_mp_future_left[si],
                    Centerline::g_mp_density[si]);
        }
        fflush(g_mp_fd);
    }
}

// Write per-tick assignment stats (called from strategy_centerline::plan)
inline void log_assign_stats(int tick, int exempt, int branch, int dirlock, int cap, int accepted) {
    if (g_assign_fd) {
        fprintf(g_assign_fd, "%d,%d,%d,%d,%d,%d\n", tick, exempt, branch, dirlock, cap, accepted);
        fflush(g_assign_fd);
    }
}

// Dump all task profiles (task_id -> errand1/errand2 main_dist/branch_dist)
// Called once at simulation end. Output enables agent->task analysis from JSON events.
inline void dump_task_profiles(SharedEnvironment* env) {
    FILE* fp = fopen("C:/Users/Administrator/.openclaw/workspace/task_profiles.csv", "w");
    if (!fp) return;
    fprintf(fp, "task_id,e1_m,e1_b,e2_m,e2_b\n");
    for (auto& kv : env->task_pool) {
        int tid = kv.first;
        auto& locs = kv.second.locations;
        if (locs.empty()) continue;
        int e1_m=-1,e1_b=-1,e2_m=-1,e2_b=-1;
        Centerline::project(locs[0], e1_m, e1_b);
        if (locs.size() >= 2) {
            Centerline::project(locs[1], e2_m, e2_b);
        }
        fprintf(fp, "%d,%d,%d,%d,%d\n", tid, e1_m, e1_b, e2_m, e2_b);
    }
    fclose(fp);
}

inline void close() {
    // Dump task profiles before closing
    if (g_env_ptr) dump_task_profiles(g_env_ptr);
#if USE_DIAG_LOG
    if (g_fd) fclose(g_fd);
    g_fd = nullptr;
    if (g_agent_fd) fclose(g_agent_fd);
    g_agent_fd = nullptr;
    if (g_mp_fd) fclose(g_mp_fd);
    g_mp_fd = nullptr;
    if (g_assign_fd) fclose(g_assign_fd);
    g_assign_fd = nullptr;
#endif
}

} // namespace CorridorLog
