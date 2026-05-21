// strategy_centerline.h — V3: zone-range + per-errand scoring
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "utils.h"
#include "config.h"
#include "centerline.h"
#include "connection_exempt.h"
#include "pibt_lock.h"
#include "idle_cap.h"
#include "corridor_log.h"
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

// ====== 统一距离函数 ======
// 返回loc_a到loc_b的最短网格路径长度
// BFS表优先 → |Δmain|+|Δbranch| → 曼哈顿
static inline int unified_dist(int loc_a, int loc_b) {
    // 1) BFS表（最精确）
    int d = BFSTable::get_dist(loc_a, loc_b);
    if (d > 0) return d;
    
    // 2) main_dist路径
    int a_m, a_b, b_m, b_b;
    Centerline::project(loc_a, a_m, a_b);
    Centerline::project(loc_b, b_m, b_b);
    if (a_m >= 0 && b_m >= 0) {
        return std::abs(b_m - a_m) + std::abs(b_b - a_b);
    }
    
    // 3) 曼哈顿（兜底）
    int cols = g_env->cols;
    return std::abs(loc_a / cols - loc_b / cols) + std::abs(loc_a % cols - loc_b % cols);
}

void init(SharedEnvironment* env) {
    g_env = env;
    Centerline::init(env);
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d, zones=%zu, cells=%zu\n",
           env->num_of_agents, env->cols, env->rows,
           Centerline::g_zone_ranges.size(), Centerline::g_centerline_cells.size());
    
    CorridorLog::init();
    CorridorLog::set_env(env);
    static bool atexit_reg = false;
    if (!atexit_reg) {
        atexit_reg = true;
        std::atexit([]() { CorridorLog::close(); });
    }
}

// Per-errand base_score using zone→main range overlap
static inline float base_score(int a_loc, int tid, int step, int start_idx = 0) {
    (void)step;
    auto it = g_env->task_pool.find(tid);
    if (it == g_env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;
    auto& locs = it->second.locations;

    // === Future flow exemption: skip direction lock if no main point along path has high future flow ===
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
                for (int si = 0; si < Centerline::g_mp_count; si++) {
                    int sm = Centerline::g_mp_main_dist[si];
                    if (sm < 0) continue;
                    if (sm < lo || sm > hi) continue;
                    float ftotal = Centerline::g_mp_future_right[si] + Centerline::g_mp_future_left[si];
                    float flow_th_ex = FUTURE_FLOW_LOCK_TH + Centerline::g_mp_width_bonus[si];
#if USE_ACTIVE_RATIO_RELAX
                    if (step >= ACTIVE_RELAX_DELAY_TICKS && Centerline::g_mp_density[si] < ACTIVE_RATIO_DENSITY_MAX) {
                        float active = CenterlineAccess::active_ratio;
                        if (active < ACTIVE_RATIO_BASELINE) {
                            float factor = (ACTIVE_RATIO_BASELINE - active) / ACTIVE_RATIO_BASELINE;
#if ACTIVE_RATIO_RELAX_EXP >= 2
                            float _base = factor;
                            for (int _e = 1; _e < ACTIVE_RATIO_RELAX_EXP; _e++) factor *= _base;
#endif
                            flow_th_ex += factor * (FUTURE_FLOW_TH_MAX - FUTURE_FLOW_LOCK_TH);
                        }
                    }
#endif
                    if (ftotal >= flow_th_ex) {
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
                    int d = unified_dist(prev, loc);
                    total += 1.0f / (float)(d + 1);
                    prev = loc;
                }
                return total;
            }
        }
    }

    // === Branch task check: BFS table (No Man Sky approach) ===
    {
        bool is_branch = true;
        int check_prev = a_loc;
        for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
            int loc = locs[ei];
            if (!BFSTable::is_branch_path(check_prev, loc)) {
                is_branch = false;
                break;
            }
            check_prev = loc;
        }
        
        if (is_branch) {
            // Pure branch task: unified distance
            float total = 0;
            int prev = a_loc;
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int d = unified_dist(prev, loc);
                total += 1.0f / (float)(d + 1);
                prev = loc;
            }
            return total;
        }

        // === Escape exemption ===
        {
            int nzc = Centerline::g_n24c;
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
        float best_align = 1.0f;  // 1.0 = perfectly aligned, lower = against flow
        // Direction lock based on 1D main point density
        for (int si = 0; si < Centerline::g_mp_count; si++) {
            int si_m = Centerline::g_mp_main_dist[si];
            if (si_m < 0) continue;
            if (si_m < lo) continue;
            if (si_m > hi) continue;  // main point is within the path segment
            int d = Centerline::g_mp_density[si];
            if (Centerline::g_mp_lock_dir[si] != 0 && Centerline::g_mp_lock_dir[si] != intent) {
                return -1e9f;
            }
            if (USE_DENSITY_CAP_80 && d >= DENSITY_CAP_80) {
                return -1e9f;
            }
            if (d > d_max_seg) d_max_seg = d;
            // Future flow alignment: prefer tasks WITH predicted traffic
            float fr = Centerline::g_mp_future_right[si];
            float fl = Centerline::g_mp_future_left[si];
            float total = fr + fl;
            if (total >= 2.0f) {
                float align = (intent > 0) ? (fr / total) : (fl / total);
                if (align < best_align) best_align = align;
            }
        }

        float seg_dist = (float)(std::abs(prev_m - tm) + std::abs(prev_b - tb));
        // Penalize task1->task2 segment less (agents already committed)
        float seg_cost = (ei > (size_t)start_idx) ? (seg_dist * 0.3f + 1.0f) : (seg_dist + 1.0f);
        float seg_score = 1.0f / seg_cost;
        if (seg_cost > MAX_TASK_SEGMENT_DIST) {
            float penalty = (float)MAX_TASK_SEGMENT_DIST / seg_cost;
            seg_score *= penalty * penalty;  // quadratic penalty, never blocks completely
        }

        if (d_max_seg > ROUTE_THRESHOLD) {
            int idx = (d_max_seg > 40) ? 40 : d_max_seg;
            seg_score /= DIVISOR_TABLE[idx - ROUTE_THRESHOLD];
        }
        
        // Future flow alignment: penalize tasks against predicted traffic
        if (best_align < 0.7f) {
            float pen = (0.7f - best_align) * 0.5f;  // align=0.7→0%, align=0.4→15%, align=0.0→35%
            if (pen > 0.35f) pen = 0.35f;
            seg_score *= (1.0f - pen);
        }

        total_score += seg_score;
        prev_m = tm; prev_b = tb;
    }

    return total_score;
}

void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_mp_density(env);
    
    // Compute active agent ratio (fraction of agents with tasks)
    {
        int active = 0;
        for (int a = 0; a < env->num_of_agents; a++)
            if (env->curr_task_schedule[a] >= 0) active++;
        CenterlineAccess::active_ratio = (float)active / (float)env->num_of_agents;
    }
    
    // Compute future flow: agent pos→task path projection, distance-weighted
    std::fill(Centerline::g_mp_future_right.begin(), Centerline::g_mp_future_right.end(), 0.0f);
    std::fill(Centerline::g_mp_future_left.begin(), Centerline::g_mp_future_left.end(), 0.0f);
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        int m_curr = -1, b_curr;
        Centerline::project(a_loc, m_curr, b_curr);
        if (m_curr < 0) continue;
        int prev_m = m_curr;
        for (int loc : it->second.locations) {
            int tm, tb;
            Centerline::project(loc, tm, tb);
            if (tm < 0) continue;
            int lo = std::min(prev_m, tm);
            int hi = std::max(prev_m, tm);
            int dir = (tm > prev_m) ? 1 : -1;
            for (int si = 0; si < Centerline::g_mp_count; si++) {
                int sm = Centerline::g_mp_main_dist[si];
                if (sm < lo || sm > hi) continue;
                if ((dir > 0 && sm <= m_curr) || (dir < 0 && sm >= m_curr)) continue;
                float d_main = (float)std::abs(sm - m_curr);
                float adj_branch = (float)(b_curr > FUTURE_FLOW_BRANCH_OFFSET ? b_curr - FUTURE_FLOW_BRANCH_OFFSET : 0);
                float d_branch = FUTURE_FLOW_BRANCH_PENALTY * adj_branch;
                float w = 1.0f / (1.0f + d_main + d_branch);
                if (dir > 0) Centerline::g_mp_future_right[si] += w;
                else Centerline::g_mp_future_left[si] += w;
            }
            prev_m = tm;
        }
    }
    
    // Future flow from idle agents going to spawn (dummy goal)
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid >= 0) continue;  // skip agents with tasks (already handled above)
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        int spawn_loc = env->start_states.at(a).location;
        if (spawn_loc < 0 || spawn_loc == a_loc) continue;  // already at spawn
        int m_curr, b_curr;
        Centerline::project(a_loc, m_curr, b_curr);
        if (m_curr < 0) continue;
        int tm, tb;
        Centerline::project(spawn_loc, tm, tb);
        if (tm < 0) continue;
        int lo = std::min(m_curr, tm);
        int hi = std::max(m_curr, tm);
        int dir = (tm > m_curr) ? 1 : -1;
        for (int si = 0; si < Centerline::g_mp_count; si++) {
            int sm = Centerline::g_mp_main_dist[si];
            if (sm < lo || sm > hi) continue;
            if ((dir > 0 && sm <= m_curr) || (dir < 0 && sm >= m_curr)) continue;
            float d_main = (float)std::abs(sm - m_curr);
            float adj_branch = (float)(b_curr > FUTURE_FLOW_BRANCH_OFFSET ? b_curr - FUTURE_FLOW_BRANCH_OFFSET : 0);
            float d_branch = FUTURE_FLOW_BRANCH_PENALTY * adj_branch;
            float w = 1.0f / (1.0f + d_main + d_branch);
            if (dir > 0)
                Centerline::g_mp_future_right[si] += w;
            else
                Centerline::g_mp_future_left[si] += w;
        }
    }
    
    Centerline::compute_future_direction();
    Centerline::update_mp_direction(env);
    
    // Sync centerline data to shared pointers for PIBT wall-hugging
    CenterlineAccess::g_branch_dist = &Centerline::g_branch_dist;
    CenterlineAccess::g_main_dist = &Centerline::g_main_dist;
    CenterlineAccess::g_zone_is_main = &Centerline::g_zone_is_main;
    CenterlineAccess::g_mp_loc = &Centerline::g_mp_loc;
    CenterlineAccess::g_mp_main_dist = &Centerline::g_mp_main_dist;
    CenterlineAccess::g_mp_count = Centerline::g_mp_count;
    CenterlineAccess::g_n24c = Centerline::g_n24c;
    CenterlineAccess::g_n24r = Centerline::g_n24r;
    
    CorridorLog::log_tick(env);
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
                proposed_schedule[a] = -1;
                if (USE_HOT_ZONE_EXEMPT && is_hot_zone_exempt(a_loc, env)) {
                    PIBT_Lock::g_agent_blocked[a] = 0; // ????:??
                } else {
                    PIBT_Lock::g_agent_blocked[a] = 1; // ???:???
                }
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

        // ===== Idle cap: 8-bucket uniform capacity control =====
        int idle_budgets[IDLE_NUM_BUCKETS] = {0};
        if (USE_IDLE_CAP) {
            IdleCap::compute_budgets(env, idle_budgets);
            std::shuffle(free_agents.begin(), free_agents.end(),
                         std::mt19937(std::random_device{}()));
        }

        // ===== Garbage pile: defer bottom 15%% of free tasks by internal distance =====
        std::unordered_set<int> deferred_tasks;
        if ((int)free_tasks.size() > 10) {
            std::vector<int> sorted_free(free_tasks.begin(), free_tasks.end());
            std::unordered_map<int, int> task_internal_dist;
            for (int tid : free_tasks) {
                auto it = env->task_pool.find(tid);
                if (it == env->task_pool.end() || it->second.locations.size() < 2) {
                    task_internal_dist[tid] = 0; continue;
                }
                int m1, b1, m2, b2;
                Centerline::project(it->second.locations[0], m1, b1);
                Centerline::project(it->second.locations[1], m2, b2);
                task_internal_dist[tid] = (m1 >= 0 && m2 >= 0) ?
                    std::abs(m1 - m2) + std::abs(b1 - b2) : 99999;
            }
            std::sort(sorted_free.begin(), sorted_free.end(), [&](int a, int b) {
                return task_internal_dist[a] < task_internal_dist[b];
            });
            int deferred_cnt = std::max(1, (int)(sorted_free.size() * GARBAGE_TASK_RATIO));
            for (int i = (int)sorted_free.size() - deferred_cnt; i < (int)sorted_free.size(); i++)
                deferred_tasks.insert(sorted_free[i]);
        }

        std::unordered_set<int> done_tasks;
        for (int a : free_agents) {
            if (proposed_schedule[a] >= 0) continue;

            int a_loc = env->curr_states[a].location;
            int a_zone = zid(a_loc, cols);

            // ===== Idle cap: check bucket budget =====
            if (USE_IDLE_CAP) {
                int b = IdleCap::get_bucket(env, a);
                if (idle_budgets[b] <= 0) {
                    proposed_schedule[a] = -1;
                    continue;
                }
            }

            int best_tid = -1;
            float best_score = -1e9f;

            // Same zone (skip garbage if possible, fallback otherwise)
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end()) {
                float best_fallback = -1e9f; int fallback_tid = -1;
                for (int tid : sz->second) {
                    if (done_tasks.count(tid)) continue;
                    float bs = base_score(a_loc, tid, step);
                    if (bs > best_score) {
                        if (!deferred_tasks.count(tid)) {
                            best_score = bs; best_tid = tid;
                        } else if (bs > best_fallback) {
                            best_fallback = bs; fallback_tid = tid;
                        }
                    }
                }
                if (best_tid < 0 && fallback_tid >= 0) {
                    best_tid = fallback_tid; best_score = best_fallback;
                }
            }

            // Adjacent zones
            if (best_tid < 0) {
                float best_fallback = -1e9f; int fallback_tid = -1;
                for (auto& kv : tasks_by_zone) {
                    int z2 = kv.first;
                    if (z2 == a_zone || zh_dist(a_zone, z2) > 1) continue;
                    int checked = 0;
                    for (int tid : kv.second) {
                        if (done_tasks.count(tid)) continue;
                        if (checked++ >= 20) break;
                        float bs = base_score(a_loc, tid, step);
                        if (bs > best_score) {
                            if (!deferred_tasks.count(tid)) {
                                best_score = bs; best_tid = tid;
                            } else if (bs > best_fallback) {
                                best_fallback = bs; fallback_tid = tid;
                            }
                        }
                    }
                }
                if (best_tid < 0 && fallback_tid >= 0) {
                    best_tid = fallback_tid; best_score = best_fallback;
                }
            }

            if (best_tid >= 0) {
                proposed_schedule[a] = best_tid;
                // Per-assignment future flow update (feeds next tick's lock computation)
                {
                    int m_c, b_c;
                    Centerline::project(env->curr_states[a].location, m_c, b_c);
                    if (m_c >= 0) {
                        auto tsk = env->task_pool.find(best_tid);
                        if (tsk != env->task_pool.end() && !tsk->second.locations.empty()) {
                            int prv = m_c;
                            for (int loc : tsk->second.locations) {
                                int tm2, tb2;
                                Centerline::project(loc, tm2, tb2);
                                if (tm2 < 0) continue;
                                int lo2 = (prv < tm2) ? prv : tm2;
                                int hi2 = (prv > tm2) ? prv : tm2;
                                int dir2 = (tm2 > prv) ? 1 : -1;
                                for (int si = 0; si < Centerline::g_mp_count; si++) {
                                    int sm2 = Centerline::g_mp_main_dist[si];
                                    if (sm2 < lo2 || sm2 > hi2) continue;
                                    if ((dir2 > 0 && sm2 <= m_c) || (dir2 < 0 && sm2 >= m_c)) continue;
                                    float d_main = (float)((sm2 > m_c) ? (sm2 - m_c) : (m_c - sm2));
                                    float adj_branch = (float)(b_c > FUTURE_FLOW_BRANCH_OFFSET ? b_c - FUTURE_FLOW_BRANCH_OFFSET : 0);
                                    float d_branch = FUTURE_FLOW_BRANCH_PENALTY * adj_branch;
                                    float w = 1.0f / (1.0f + d_main + d_branch);
                                    if (dir2 > 0)
                                        Centerline::g_mp_future_right[si] += w;
                                    else
                                        Centerline::g_mp_future_left[si] += w;
                                    // Instant lock: recompute lock_dir right after adding flow
                                    float ftotal = Centerline::g_mp_future_right[si] + Centerline::g_mp_future_left[si];
                                    float flow_th_inst = FUTURE_FLOW_LOCK_TH + Centerline::g_mp_width_bonus[si];
#if USE_ACTIVE_RATIO_RELAX
                                    if (step >= ACTIVE_RELAX_DELAY_TICKS && Centerline::g_mp_density[si] < ACTIVE_RATIO_DENSITY_MAX) {
                                        float active = CenterlineAccess::active_ratio;
                                        if (active < ACTIVE_RATIO_BASELINE) {
                                            float factor = (ACTIVE_RATIO_BASELINE - active) / ACTIVE_RATIO_BASELINE;
#if ACTIVE_RATIO_RELAX_EXP >= 2
                                            float _base = factor;
                                            for (int _e = 1; _e < ACTIVE_RATIO_RELAX_EXP; _e++) factor *= _base;
#endif
                                            flow_th_inst += factor * (FUTURE_FLOW_TH_MAX - FUTURE_FLOW_LOCK_TH);
                                        }
                                    }
#endif
                                    if (ftotal >= flow_th_inst) {
                                        int ndir = (Centerline::g_mp_future_right[si] > Centerline::g_mp_future_left[si]) ? +1 : -1;
                                        for (int d = -FUTURE_FLOW_LOCK_SPREAD; d <= FUTURE_FLOW_LOCK_SPREAD; d++) {
                                            int ni = si + d;
                                            if (ni >= 0 && ni < Centerline::g_mp_count)
                                                Centerline::g_mp_lock_dir[ni] = ndir;
                                        }
                                    }
                                }
                                prv = tm2;
                            }
                        }
                    }
                }
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
                if (USE_IDLE_CAP) {
                    int b = IdleCap::get_bucket(env, a);
                    idle_budgets[b]--;
                }
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





