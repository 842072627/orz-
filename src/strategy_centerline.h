// strategy_centerline.h â€” V3: zone-range + per-errand scoring
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include "connection_exempt.h"
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
static std::vector<int> g_parked_task;  // per-agent parked second errand task id, -1=none

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
static inline float base_score(int a_loc, int tid, int step, int start_idx = 0) {
    (void)step;
    auto it = g_env->task_pool.find(tid);
    if (it == g_env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;
    auto& locs = it->second.locations;

    // === 1D main zone exemption: skip direction lock if all main zones along path are cool ===
    {
        int a_m, a_b;
        Centerline::project(a_loc, a_m, a_b);
        if (a_m >= 0) {
            bool exempt = true;
            int prev_m = a_m;
            for (auto& loc : locs) {
                int tm, tb;
                Centerline::project(loc, tm, tb);
                if (tm < 0) continue;
                int lo = std::min(prev_m, tm);
                int hi = std::max(prev_m, tm);
                for (auto& zr : Centerline::g_zone_ranges) {
                    if (zr.min_m <= hi && zr.max_m >= lo &&
                        Centerline::g_zone_is_main[zr.zone_id] &&
                        Centerline::g_zone24_density[zr.zone_id] >= EXEMPT_MAIN_THR) {
                        exempt = false;
                        break;
                    }
                }
                if (!exempt) break;
                prev_m = tm;
            }
            if (exempt) {
                float total = 0;
                int prev = a_loc;
                for (auto& loc : locs) {
                    int r1 = prev / g_env->cols, c1 = prev % g_env->cols;
                    int r2 = loc / g_env->cols, c2 = loc % g_env->cols;
                    float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
                    total += 1.0f / (dist + 1.0f);
                    prev = loc;
                }
                return total;
            }
        }
    }

    // === Branch task check: ALL errands off-hot-main + obstacle-free line ===
    {
        bool is_branch = true;
        int check_prev = a_loc;
        int nzc = Centerline::g_n24c;
        int max_z = (int)Centerline::g_zone_is_main.size();
        for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
            int loc = locs[ei];
            int cols = g_env->cols;
            int x1 = check_prev % cols, y1 = check_prev / cols;
            int x2 = loc % cols, y2 = loc / cols;
            // Start zone check
            int z1 = (y1 / 24) * nzc + (x1 / 24);
            if (z1 >= 0 && z1 < max_z) {
                int d1 = Centerline::g_zone24_density[z1];
                if (Centerline::g_zone_is_main[z1]) {
                    if (d1 >= ZONE_HARD_CAP)
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
                    if (d2 >= ZONE_HARD_CAP)
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
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int r1 = prev / g_env->cols, c1 = prev % g_env->cols;
                int r2 = locs[ei] / g_env->cols, c2 = locs[ei] % g_env->cols;
                int z2 = (r2 / 24) * nzc + (c2 / 24);
                if (z2 >= 0 && z2 < max_z && Centerline::g_zone24_density[z2] >= 70)
                    return -1e9f;
                float dist = std::sqrt((float)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
                total += 1.0f / (dist + 1.0f);
                prev = locs[ei];
            }
            return total;
        }

        // === Escape exemption: compute only remaining errands ===
        if (start_idx == 0) {
            float esc_score = connection_exempt_score(a_loc, locs,
                Centerline::g_zone24_density, Centerline::g_zone_is_main,
                g_env->map, g_env->cols, nzc);
            if (esc_score > 0) return esc_score;
        } else {
            std::vector<int> remaining(locs.begin() + start_idx, locs.end());
            float esc_score = connection_exempt_score(a_loc, remaining,
                Centerline::g_zone24_density, Centerline::g_zone_is_main,
                g_env->map, g_env->cols, nzc);
            if (esc_score > 0) return esc_score;
        }
    }

    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
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
                int d = Centerline::g_zone24_density[zr.zone_id];
                // Direction lock: block tasks against flow
                if (ORZ_DIRECTION_LOCK && zr.lock_dir != 0 && zr.lock_dir != intent)
                    return -1e9f;
                // Hard density cap: block ALL tasks through severely congested zones
                if (USE_DENSITY_CAP_80 && d >= DENSITY_CAP_80)
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
    Centerline::update_zone_threshold();
    Centerline::update_zone_direction(env);
}

void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    auto t0 = std::chrono::steady_clock::now();
    pre_tick_updates(env);
    auto t_pre = std::chrono::steady_clock::now();

    // Init parked task tracking
    if ((int)g_parked_task.size() != env->num_of_agents)
        g_parked_task.assign(env->num_of_agents, -1);
    // Sync parked tasks to PIBT shared state
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_parked_task[a] = g_parked_task[a];
    // Reset PIBT block flags
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_agent_blocked[a] = 0;

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
        
        // Check parked second errand first
        int parked = (a < (int)g_parked_task.size()) ? g_parked_task[a] : -1;
        if (parked >= 0) {
            int a_loc = env->curr_states[a].location;
            float bs = base_score(a_loc, parked, env->curr_timestep, 2);
            if (bs > 0) {
                proposed_schedule[a] = parked;
                g_parked_task[a] = -1;  // clear park
                free_tasks.erase(parked);
                it = free_agents.erase(it);
            } else {
                // keep parked, agent waits
                proposed_schedule[a] = -1;
                PIBT_Lock::g_agent_blocked[a] = 1;  // PIBT force wait
                ++it;
            }
            continue;
        }
        
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
                // Park second errand for multi-location tasks
                auto tit = env->task_pool.find(best_tid);
                if (tit != env->task_pool.end()) {
                    if (tit->second.locations.size() > 2)
                        g_parked_task[a] = best_tid;
                    else
                        g_parked_task[a] = -1;  // clear stale park
                }
                done_tasks.insert(best_tid);
                phase2_assigned++;
            } else {
                proposed_schedule[a] = -1;
            }
        }
    }

    for (int a : free_agents) {
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
    }
    
    // Timeout detection
    static int timeout_counter = 0;
    if (std::chrono::steady_clock::now() > endtime) {
        timeout_counter++;
        if (timeout_counter % 10 == 1) {
            printf("[SCHEDULE TIMEOUT #%d] step=%d, free_agents=%zu, phase2_assigned=%d\n",
                   timeout_counter, step, free_agents.size(), phase2_assigned);
        }
    }
}

} // namespace strategy_centerline





