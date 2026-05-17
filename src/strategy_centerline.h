// strategy_centerline.h â€” V3: zone-range + per-errand scoring
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include "pibt_lock.h"
#include "cold_zone_pushout.h"
#include "narrow_passage.h"
#include "control_zone.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <random>
#include <vector>
#include <cstdint>
#include <chrono>
#include <climits>
#include <cmath>

namespace strategy_centerline {

// ====== PIBT recursive search depth (from config.h) ======
// Used in my_pibt.cpp via extern; also constexpr here for easy reference.
#define WITHIN_SAME_ZONE(a_si, t_si)  ((a_si) >= 0 && (t_si) == (a_si))

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
    NarrowPassage::init(env);
    ControlZone::init(env);
    printf("CENTERLINE_SCHEDULER V5: agents=%d, map=%dx%d, squares=%zu, cells=%zu\n",
           env->num_of_agents, env->cols, env->rows,
           Centerline::g_main_squares.size(), Centerline::g_centerline_cells.size());
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
                for (auto& sq : Centerline::g_main_squares) {
                    if (sq.min_m <= hi && sq.max_m >= lo &&
                        sq.density >= ZONE_HARD_CAP) {
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
        int max_z = (int)Centerline::g_zone24_density.size();
        for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
            int loc = locs[ei];
            int cols = g_env->cols;
            int x1 = check_prev % cols, y1 = check_prev / cols;
            int x2 = loc % cols, y2 = loc / cols;
            // Start zone check
            int z1 = (y1 / 24) * nzc + (x1 / 24);
            if (z1 >= 0 && z1 < max_z) {
                int d1 = Centerline::g_zone24_density[z1];
                if (Centerline::is_main_cell(check_prev)) {
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
                if (Centerline::is_main_cell(loc)) {
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

    }

    // === (connection_exempt removed — cold zone pushout handles escape) ===

    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;

    // ====== Same-zone exemption: agent and task share a main zone → skip direction lock & density cap ======
    // 如果 agent 当前位置和 task 在同一个 main square（或1D投影落在同一个square范围），
    // 那么 agent 不应被推到冷区，应当去做任务。不然永远被推来推去。
    bool same_zone_exempt = false;
    int a_si = (a_loc >= 0 && a_loc < (int)Centerline::g_cell_square.size()) ? Centerline::g_cell_square[a_loc] : -1;
    if (a_si >= 0) {
        // 方法1: same cell square index
        for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
            int t_si = (locs[ei] >= 0 && locs[ei] < (int)Centerline::g_cell_square.size()) ? Centerline::g_cell_square[locs[ei]] : -1;
            if (t_si == a_si) { same_zone_exempt = true; break; }
        }
        // 方法2: 1D投影落在同一个 square 的 main_dist 范围内
        if (!same_zone_exempt) {
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int tm2, tb2;
                Centerline::project(locs[ei], tm2, tb2);
                if (tm2 >= 0 && tm2 >= Centerline::g_main_squares[a_si].min_m &&
                    tm2 <= Centerline::g_main_squares[a_si].max_m) {
                    same_zone_exempt = true; break;
                }
            }
        }
    }

    float total_score = 0;
    int prev_m = a_m, prev_b = a_b;

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        Centerline::project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        // Square overlap check + direction lock (V4: path-aligned squares)
        int d_max_seg = 0;
        // 用g_cell_square找到agent所在的MainSquare（与base_score顶部同区检测一致）
        int agent_sq_idx = Centerline::g_cell_square[a_loc];
        for (size_t si = 0; si < Centerline::g_main_squares.size(); si++) {
            auto& sq = Centerline::g_main_squares[si];
            if (sq.min_m <= hi && sq.max_m >= lo) {
                int d = sq.density;  // V4: square density (not grid zone)
                // 同区豁免：只豁免agent所在的方块（g_cell_square匹配），路径上的其他方块仍需遵守方向锁
                bool sq_is_agent_zone = same_zone_exempt && agent_sq_idx >= 0 && (int)si == agent_sq_idx;
                if (!sq_is_agent_zone) {
                    // Direction lock: block tasks against flow
                    if (ORZ_DIRECTION_LOCK && sq.lock_dir != 0 && sq.lock_dir != intent)
                        return -1e9f;
                }
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
    Centerline::update_square_density(env);
    Centerline::update_square_direction(env);
}

void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    auto t0 = std::chrono::steady_clock::now();
    pre_tick_updates(env);
    
    // Edge lane pushout (new v5 architecture): 每tick管控决策
    if (USE_COLD_ZONE_PUSHOUT) {
        ControlZone::update(env);
    }
    
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
                // Parked task not feasible → let PIBT wait (don't block the agent)
                proposed_schedule[a] = -1;
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

    if (free_agents.empty()) { return; }

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
                        g_parked_task[a] = -1;
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





