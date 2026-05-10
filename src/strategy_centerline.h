// strategy_centerline.h â€” V3: zone-range + per-errand scoring
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

namespace strategy_centerline {

static constexpr int ROUTE_THRESHOLD = 20;
static constexpr float DIVISOR_TABLE[21] = {
    1.0f, 1.1f, 1.2f, 1.3f, 1.4f,
    1.5f, 1.6f, 1.7f, 1.8f, 1.9f,
    2.0f, 2.1f, 2.2f, 2.3f, 2.4f,
    2.5f, 2.6f, 2.7f, 2.8f, 2.9f,
    3.0f
};

static SharedEnvironment* g_env = nullptr;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
static inline int zh_dist(int z1, int z2) {
    return std::abs(z1/100 - z2/100) + std::abs(z1%100 - z2%100);
}

void init(SharedEnvironment* env) {
    g_env = env;
    Centerline::init(env);
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d, zones=%zu, cells=%zu\n",
           env->num_of_agents, env->cols, env->rows,
           Centerline::g_zone_ranges.size(), Centerline::g_centerline_cells.size());
}

// Per-errand base_score using zoneâ†’main range overlap
static inline float base_score(int a_loc, int tid, int step) {
    (void)step;
    auto it = g_env->task_pool.find(tid);
    if (it == g_env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;

    // === Branch task check: ALL errands off-hot-main + obstacle-free line ===
    {
        bool is_branch = true;
        int check_prev = a_loc;
        int nzc = Centerline::g_n24c;
        int max_z = (int)Centerline::g_zone_is_main.size();
        for (int loc : it->second.locations) {
            int cols = g_env->cols;
            int x1 = check_prev % cols, y1 = check_prev / cols;
            int x2 = loc % cols, y2 = loc / cols;
            // Start zone check
            int z1 = (y1 / 24) * nzc + (x1 / 24);
            if (z1 >= 0 && z1 < max_z) {
                int d1 = Centerline::g_zone24_density[z1];
                if (Centerline::g_zone_is_main[z1]) {
                    if (d1 >= Centerline::CONTAIN_LOCK_DENSITY || d1 >= Centerline::HOT_BLOCK_DENSITY)
                        { is_branch = false; break; }
                } else {
                    if (d1 >= 70) { is_branch = false; break; }
                }
            }
            // End zone check
            int z2 = (y2 / 24) * nzc + (x2 / 24);
            if (z2 >= 0 && z2 < max_z) {
                int d2 = Centerline::g_zone24_density[z2];
                if (Centerline::g_zone_is_main[z2]) {
                    if (d2 >= Centerline::CONTAIN_LOCK_DENSITY || d2 >= Centerline::HOT_BLOCK_DENSITY)
                        { is_branch = false; break; }
                } else {
                    if (d2 >= 70) { is_branch = false; break; }
                }
            }
            // Bresenham line-of-sight
            int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
            int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
            int err = dx + dy;
            int cx = x1, cy = y1;
            while (true) {
                if (g_env->map[cy * cols + cx] != 0) { is_branch = false; break; }
                if (cx == x2 && cy == y2) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; cx += sx; }
                if (e2 <= dx) { err += dx; cy += sy; }
            }
            if (!is_branch) break;
            check_prev = loc;
        }
        
        if (is_branch) {
            // Branch task: distance scoring, only block at >= 70
            float total = 0;
            int prev = a_loc;
            for (int loc : it->second.locations) {
                int r1 = prev / g_env->cols, c1 = prev % g_env->cols;
                int r2 = loc / g_env->cols, c2 = loc % g_env->cols;
                int z2 = (r2 / 24) * nzc + (c2 / 24);
                if (z2 >= 0 && z2 < max_z && Centerline::g_zone24_density[z2] >= 70)
                    return -1e9f;
                float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
                total += 1.0f / (dist + 1.0f);
                prev = loc;
            }
            return total;
        }
    }

    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;
    auto& locs = it->second.locations;

    for (size_t ei = 0; ei < locs.size(); ei++) {
        int tm, tb;
        Centerline::project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        // Zone overlap check + direction lock (any seg locked = whole task blocked)
        int d_max_seg = 0;
        for (auto& zr : Centerline::g_zone_ranges) {
            if (zr.min_m <= hi && zr.max_m >= lo) {
                // Direction lock check (config-controlled)
                if (ORZ_DIRECTION_LOCK && zr.lock_dir != 0 && zr.lock_dir != intent)
                    return -1e9f;
                // HOT_BLOCK: density >= 23 blocks ALL assignments through this zone
                int d = Centerline::g_zone24_density[zr.zone_id];
                if (d >= Centerline::HOT_BLOCK_DENSITY)
                    return -1e9f;
                if (d > d_max_seg) d_max_seg = d;
            }
        }

        float seg_cost = (float)(std::abs(prev_m - tm) + std::abs(prev_b - tb)) + 1.0f;
        float seg_score = 1.0f / seg_cost;

        if (d_max_seg > ROUTE_THRESHOLD) {
            int idx = (d_max_seg > 40) ? 40 : d_max_seg;
            seg_score /= DIVISOR_TABLE[idx - ROUTE_THRESHOLD];
        }

        total_score += seg_score;
        prev_m = tm; prev_b = tb;
    }

    return total_score;
}

void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_zone_direction(env);
}

void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    auto t0 = std::chrono::steady_clock::now();
    pre_tick_updates(env);
    auto t_pre = std::chrono::steady_clock::now();

    // PIBT direction block (disabled by config)
    if (ORZ_PIBT_DIRECTION_BLOCK) {
        int n24c = (env->cols + 23) / 24;
        for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++) {
            PIBT_Lock::g_agent_blocked[a] = 0;
        }
    }

    TimePoint endtime = t0 + std::chrono::milliseconds(limit);
    int cols = env->cols;
    static int s_step = 0;
    int step = s_step++;

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

    // Phase 2: Greedy assignment
    std::unordered_map<int, std::vector<int>> tasks_by_zone;
    int phase2_assigned = 0;
    if (!free_agents.empty() && !free_tasks.empty() && std::chrono::steady_clock::now() <= endtime) {
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

            // Adjacent zones
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
                phase2_assigned++;
            } else {
                // Debug: check WHY first free agent gets -1 (do once)
                static int s_debugged = 0;
                if (a == 0 && !s_debugged && step >= 4) {
                    s_debugged = 1;
                    int probe_tid = -1;
                    for (auto& kv : tasks_by_zone) {
                        for (int tid : kv.second) {
                            if (!done_tasks.count(tid)) { probe_tid = tid; break; }
                        }
                        if (probe_tid >= 0) break;
                    }
                    if (probe_tid >= 0) {
                        float bs = base_score(a_loc, probe_tid, step);
                        printf("[DEBUG] tick=%d a=%d loc=%d bs=%.1f\n",
                               step, a, a_loc, bs);
                        
                    }
                }
                proposed_schedule[a] = -1;
            }
        }
    }

    for (int a : free_agents) {
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
    }
    
    // WHY-IT-STOPPED logging
    if (step >= 40 && step % 10 == 0) {
        // Check if Phase 2 is running or being skipped
        int a_eg = free_agents.empty() ? -1 : free_agents[0];
        int a_loc = (a_eg >= 0) ? env->curr_states[a_eg].location : -1;
        int a_z = (a_eg >= 0) ? zid(a_loc, cols) : -1;
        int n_blocks = 0, n_locked = 0;
        int hot18 = 0;
        for (auto& zr : Centerline::g_zone_ranges) {
            int d = Centerline::g_zone24_density[zr.zone_id];
            if (d >= Centerline::HOT_BLOCK_DENSITY) hot18++;
            if (zr.lock_dir != 0) { n_blocks++; if(zr.lock_dir) n_locked++; }
        }
        printf("[WHY] tick=%d free=%d tasks=%d p2_assigned=%d a_eg=%d a_z=%d hot18=%d locked_zones=%d tasks_by_zone=%zu\n",
               step, (int)free_agents.size(), (int)free_tasks.size(), phase2_assigned, a_eg, a_z, hot18, n_locked, 0U);
    }
    
    auto t_end = std::chrono::steady_clock::now();
    int elapsed = (int)std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t0).count();
    int pre_ms = (int)std::chrono::duration_cast<std::chrono::microseconds>(t_pre - t0).count();
    if (step < 200 || step % 200 == 0 || elapsed >= limit) {
        printf("[TIMING] tick=%d limit=%dms sched=%dms pre=%dus agents=%d free=%d tasks=%d\n",
               step, limit, elapsed, pre_ms, env->num_of_agents, (int)free_agents.size(), (int)free_tasks.size());
    }
    // Dump zone densities at key ticks
    if (step < 100) {
        int max_d = 0, max_z = -1;
        int over14 = 0, over20 = 0;
        for (size_t zi = 0; zi < Centerline::g_zone24_density.size(); zi++) {
            int d = Centerline::g_zone24_density[zi];
            if (d > max_d) { max_d = d; max_z = (int)zi; }
            if (d >= 14) over14++;
            if (d >= 20) over20++;
        }
        printf("[DENSITY] tick=%d max_z=%d max_d=%d over14=%d over20=%d\n",
               step, max_z, max_d, over14, over20);
    }
}

} // namespace strategy_centerline





