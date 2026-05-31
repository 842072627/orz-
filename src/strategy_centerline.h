// strategy_centerline.h — V3: zone-range + per-errand scoring
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
static std::vector<int> g_parked_task;

static int g_dbg_exempt = 0;
static int g_dbg_branch = 0;
static int g_dbg_dirlock = 0;
static int g_dbg_accepted = 0;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
static inline int zh_dist(int z1, int z2) {
    return std::abs(z1/100 - z2/100) + std::abs(z1%100 - z2%100);
}

static inline int unified_dist(int loc_a, int loc_b) {
    int d = BFSTable::get_dist(loc_a, loc_b);
    if (d > 0) return d;
    int a_m, a_b, b_m, b_b;
    Centerline::project(loc_a, a_m, a_b);
    Centerline::project(loc_b, b_m, b_b);
    if (a_m >= 0 && b_m >= 0)
        return std::abs(b_m - a_m) + std::abs(b_b - a_b);
    int cols = g_env->cols;
    return std::abs(loc_a / cols - loc_b / cols) + std::abs(loc_a % cols - loc_b % cols);
}

// ====== Future flow: cumulative path distance ======
static inline void add_agent_future_flow(
    int agent_loc,
    const std::vector<int>& task_locs,
    SharedEnvironment* env,
    bool do_instant_lock = false)
{
    int m_curr, b_curr;
    Centerline::project(agent_loc, m_curr, b_curr);
    if (m_curr < 0) return;

    int prev_m = m_curr;
    int prev_loc = agent_loc;
    int cumul = 0;

    float adj_branch = (float)(b_curr > FUTURE_FLOW_BRANCH_OFFSET
                               ? b_curr - FUTURE_FLOW_BRANCH_OFFSET : 0);
    float d_branch = FUTURE_FLOW_BRANCH_PENALTY * adj_branch;

    for (int loc : task_locs) {
        int tm, tb;
        Centerline::project(loc, tm, tb);
        if (tm < 0) continue;

        // Off-main segment: whole path stays in branch → no main line flow
        if (BFSTable::is_branch_path(prev_loc, loc)) {
            cumul += std::abs(tm - prev_m);
            prev_m = tm;
            prev_loc = loc;
            continue;
        }

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int dir = (tm > prev_m) ? 1 : -1;
        int seg_len = std::abs(tm - prev_m);

        auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), lo);
        auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), hi);
        int si_lo = (int)(lo_it - Centerline::g_mp_main_dist.begin());
        int si_hi = (int)(hi_it - Centerline::g_mp_main_dist.begin());
        for (int si = si_lo; si < si_hi; si++) {
            int sm = Centerline::g_mp_main_dist[si];
            if ((dir > 0 && sm <= prev_m) || (dir < 0 && sm >= prev_m)) continue;

            int mp_offset = std::abs(sm - prev_m);
            int d_main = cumul + mp_offset;

            float w = 1.0f / (1.0f + (float)d_main + d_branch);

            if (dir > 0)
                Centerline::g_mp_future_right[si] += w;
            else
                Centerline::g_mp_future_left[si] += w;

            if (do_instant_lock) {
                float ftotal = Centerline::g_mp_future_right[si] + Centerline::g_mp_future_left[si];
                float flow_th = Centerline::get_mp_flow_th(si, env);
                if (flow_th < FF_TH_BAND_4 && ftotal >= flow_th) {
                    int ndir = (Centerline::g_mp_future_right[si] > Centerline::g_mp_future_left[si]) ? +1 : -1;
                    for (int d = -FUTURE_FLOW_LOCK_SPREAD; d <= FUTURE_FLOW_LOCK_SPREAD; d++) {
                        int ni = si + d;
                        if (ni < 0 || ni >= Centerline::g_mp_count) continue;
                        Centerline::g_mp_lock_dir[ni] = ndir;
                    }
                }
            }
        }

        cumul += seg_len;
        prev_m = tm;
        prev_loc = loc;
    }
}

void init(SharedEnvironment* env) {
    g_env = env;
    Centerline::init(env);
    // Centerline::compute_cell_width(env);  // already called inside Centerline::init()
#if USE_DIAG_LOG
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d, zones=%zu, cells=%zu\n",
           env->num_of_agents, env->cols, env->rows,
           Centerline::g_zone_ranges.size(), Centerline::g_centerline_cells.size());
#endif
#if USE_DIAG_LOG
    CorridorLog::init();
    CorridorLog::set_env(env);
    static bool atexit_reg = false;
    if (!atexit_reg) {
        atexit_reg = true;
        std::atexit([]() { CorridorLog::close(); });
    }
#endif
}

// ====== base_score ======
static inline float base_score(int a_loc, int tid, SharedEnvironment* env, int start_idx = 0) {
    auto it = env->task_pool.find(tid);
    if (it == env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;
    auto& locs = it->second.locations;

    // Future flow exemption
    {
        int a_m, a_b;
        Centerline::project(a_loc, a_m, a_b);
        if (a_m >= 0) {
            bool exempt = true;
            int prev_m = a_m;
            int a_zone = -1;
#if EXCLUDE_AGENT_ZONE
            a_zone = (a_loc / g_env->cols / 24) * Centerline::g_n24c + (a_loc % g_env->cols / 24);
#endif
            for (auto& loc : locs) {
                int tm, tb;
                Centerline::project(loc, tm, tb);
                if (tm < 0) continue;
                int lo = std::min(prev_m, tm);
                int hi = std::max(prev_m, tm);
                auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), lo);
                auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), hi);
                for (int si = (int)(lo_it - Centerline::g_mp_main_dist.begin());
                     si < (int)(hi_it - Centerline::g_mp_main_dist.begin()); si++) {
#if EXCLUDE_AGENT_ZONE
                    if (a_zone >= 0) {
                        int mloc = Centerline::g_mp_loc[si];
                        if (mloc >= 0) {
                            int mp_z = (mloc / g_env->cols / 24) * Centerline::g_n24c + (mloc % g_env->cols / 24);
                            if (mp_z == a_zone) continue;
                        }
                    }
#endif
                    float ftotal = Centerline::g_mp_future_right[si] + Centerline::g_mp_future_left[si];
                    float flow_th_ex = Centerline::get_mp_flow_th(si, env);
                    if (ftotal >= flow_th_ex) { exempt = false; break; }
                }
                if (!exempt) break;
                prev_m = tm;
            }
            if (exempt) {
                float wd = 0;
                int prev = a_loc;
                int si = 0;
                for (auto& loc : locs) {
                    int d = unified_dist(prev, loc);
                    float coeff = (si == 0) ? COEFF_AGENT_TASK1 : COEFF_TASK_SEGMENT;
                    wd += (float)d * coeff;
                    prev = loc;
                    si++;
                }
                g_dbg_exempt++;
                return 1.0f / (wd + 1.0f);
            }
        }
    }

    // Branch task check
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
            float wd = 0;
            int prev = a_loc;
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int d = unified_dist(prev, loc);
                float coeff = (ei == (size_t)start_idx) ? COEFF_AGENT_TASK1 : COEFF_TASK_SEGMENT;
                wd += (float)d * coeff;
                prev = loc;
            }
            g_dbg_branch++;
            return 1.0f / (wd + 1.0f);
        }
    }

    // ====== 统一任务评分 ======
    // 加权距离之和 → 分数
    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    if (a_m < 0) return 1.0f;
    float weighted_dist = 0.0f;
    int global_d_max = 0;
    float global_worst_align = 1.0f;
    int prev_m = a_m, prev_b = a_b;
    int prev_loc = a_loc;
    int a_zone = -1;
#if EXCLUDE_AGENT_ZONE
    a_zone = (a_loc / g_env->cols / 24) * Centerline::g_n24c + (a_loc % g_env->cols / 24);
#endif

    for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
        int tm, tb;
        Centerline::project(locs[ei], tm, tb);
        if (tm < 0) continue;

        int lo = std::min(prev_m, tm);
        int hi = std::max(prev_m, tm);
        int intent = (tm > prev_m) ? +1 : -1;

        int d_max_seg = 0;
        float best_align = 1.0f;
        auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), lo);
        auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), hi);
        for (int si = (int)(lo_it - Centerline::g_mp_main_dist.begin());
             si < (int)(hi_it - Centerline::g_mp_main_dist.begin()); si++) {
#if EXCLUDE_AGENT_ZONE
            if (a_zone >= 0) {
                int mloc = Centerline::g_mp_loc[si];
                if (mloc >= 0) {
                    int mp_z = (mloc / g_env->cols / 24) * Centerline::g_n24c + (mloc % g_env->cols / 24);
                    if (mp_z == a_zone) continue;
                }
            }
#endif
            int d = Centerline::g_mp_density[si];
            if (Centerline::g_mp_lock_dir[si] != 0 && Centerline::g_mp_lock_dir[si] != intent) {
                g_dbg_dirlock++;
                return -1e9f;
            }
            if (USE_DENSITY_HARD_CAP && d >= DENSITY_HARD_CAP)
                return -1e9f;
            if (d > d_max_seg) d_max_seg = d;
            float fr = Centerline::g_mp_future_right[si];
            float fl = Centerline::g_mp_future_left[si];
            float ftotal = fr + fl;
            if (ftotal >= 2.0f) {
                float align = (intent > 0) ? (fr / ftotal) : (fl / ftotal);
                if (align < best_align) best_align = align;
            }
        }

        // 统一加权距离累加
        float seg_dist = (float)unified_dist(prev_loc, locs[ei]);
        float coeff = (ei == (size_t)start_idx) ? COEFF_AGENT_TASK1 : COEFF_TASK_SEGMENT;
        weighted_dist += seg_dist * coeff;

        if (d_max_seg > global_d_max) global_d_max = d_max_seg;
        if (best_align < global_worst_align) global_worst_align = best_align;

        prev_m = tm; prev_b = tb;
        prev_loc = locs[ei];
    }

    // 统一评分：1 / (加权距离和 + 1)
    float final_score = 1.0f / (weighted_dist + 1.0f);
    if (weighted_dist > MAX_TASK_SEGMENT_DIST) {
        float penalty = (float)MAX_TASK_SEGMENT_DIST / weighted_dist;
        final_score *= penalty * penalty;
    }
    if (global_d_max > ROUTE_THRESHOLD) {
        int idx = (global_d_max > 40) ? 40 : global_d_max;
        final_score /= DIVISOR_TABLE[idx - ROUTE_THRESHOLD];
    }
    if (global_worst_align < 0.7f) {
        float pen = (0.7f - global_worst_align) * 0.5f;
        if (pen > 0.35f) pen = 0.35f;
        final_score *= (1.0f - pen);
    }
    return final_score;
}

// ====== Pre-tick updates ======
void pre_tick_updates(SharedEnvironment* env) {
    Centerline::update_zone24_density(env);
    Centerline::update_mp_density(env);

    {
        int active = 0;
        for (int a = 0; a < env->num_of_agents; a++)
            if (env->curr_task_schedule[a] >= 0) active++;
        CenterlineAccess::active_ratio = (float)active / (float)env->num_of_agents;
    }

    std::fill(Centerline::g_mp_future_right.begin(), Centerline::g_mp_future_right.end(), 0.0f);
    std::fill(Centerline::g_mp_future_left.begin(),  Centerline::g_mp_future_left.end(),  0.0f);

    // Agents with tasks
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        add_agent_future_flow(a_loc, it->second.locations, env);
    }

    // Idle agents returning to spawn
    for (int a = 0; a < env->num_of_agents; a++) {
        if (env->curr_task_schedule[a] >= 0) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        int spawn_loc = env->start_states.at(a).location;
        if (spawn_loc < 0 || spawn_loc == a_loc) continue;
        add_agent_future_flow(a_loc, {spawn_loc}, env);
    }

    Centerline::update_mp_direction(env);

    CenterlineAccess::g_branch_dist   = &Centerline::g_branch_dist;
    CenterlineAccess::g_main_dist     = &Centerline::g_main_dist;
    CenterlineAccess::g_zone_is_main  = &Centerline::g_zone_is_main;
    CenterlineAccess::g_mp_loc        = &Centerline::g_mp_loc;
    CenterlineAccess::g_mp_main_dist  = &Centerline::g_mp_main_dist;
    CenterlineAccess::g_mp_count      = Centerline::g_mp_count;
    CenterlineAccess::g_n24c          = Centerline::g_n24c;
    CenterlineAccess::g_n24r          = Centerline::g_n24r;

#if USE_DIAG_LOG
    CorridorLog::log_tick(env);
#endif
}

// ====== Plan ======
void plan(int limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    auto t0 = std::chrono::steady_clock::now();

    pre_tick_updates(env);

    if ((int)g_parked_task.size() != env->num_of_agents)
        g_parked_task.assign(env->num_of_agents, -1);
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_parked_task[a] = g_parked_task[a];
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++)
        PIBT_Lock::g_agent_blocked[a] = 0;

    TimePoint endtime = t0 + std::chrono::milliseconds(limit);
    int cols = env->cols;
    static int s_step = 0;
    int step = s_step++;

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
        int cur = env->curr_task_schedule[a];
        int parked = (a < (int)g_parked_task.size()) ? g_parked_task[a] : -1;
        if (parked >= 0) {
            int a_loc = env->curr_states[a].location;
            float bs = base_score(a_loc, parked, env, 2);
            if (bs > 0) {
                proposed_schedule[a] = parked;
                g_parked_task[a] = -1;
                free_tasks.erase(parked);
                it = free_agents.erase(it);
            } else {
                proposed_schedule[a] = -1;
                PIBT_Lock::g_agent_blocked[a] = 1;
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

    if (free_agents.empty()) {
        for (int a = 0; a < env->num_of_agents; a++)
            if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
        return;
    }

    // Phase 2: Greedy assignment
    std::unordered_map<int, std::vector<int>> tasks_by_zone;
    int phase2_assigned = 0;
    if (!free_agents.empty() && !free_tasks.empty() && std::chrono::steady_clock::now() <= endtime) {
        for (int tid : free_tasks) {
            if (env->task_pool.count(tid) && !env->task_pool[tid].locations.empty())
                tasks_by_zone[zid(env->task_pool[tid].locations[0], cols)].push_back(tid);
        }

        int idle_budgets[IDLE_NUM_BUCKETS] = {0};
        if (USE_IDLE_CAP) {
            IdleCap::compute_budgets(env, idle_budgets);
            std::shuffle(free_agents.begin(), free_agents.end(),
                         std::mt19937(std::random_device{}()));
        }

        // Garbage defer
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
                task_internal_dist[tid] = unified_dist(it->second.locations[0], it->second.locations[1]);
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
            if (std::chrono::steady_clock::now() > endtime) break;
            if (proposed_schedule[a] >= 0) continue;
            int a_loc = env->curr_states[a].location;
            int a_zone = zid(a_loc, cols);

            if (USE_IDLE_CAP) {
                int b = IdleCap::get_bucket(env, a);
                if (idle_budgets[b] <= 0) {
                    proposed_schedule[a] = -1;
                    continue;
                }
            }

            int best_tid = -1;
            float best_score = -1e9f;

            // Same zone
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end()) {
                float best_fallback = -1e9f; int fallback_tid = -1;
                for (int tid : sz->second) {
                    if (done_tasks.count(tid) || deferred_tasks.count(tid)) continue;
                    float bs = base_score(a_loc, tid, env);
                    if (bs > best_score) { best_score = bs; best_tid = tid; }
                    if (bs > best_fallback) { best_fallback = bs; fallback_tid = tid; }
                }
                if (best_tid < 0 && fallback_tid >= 0)
                    { best_tid = fallback_tid; best_score = best_fallback; }
            }

            // Other zones
            if (best_tid < 0) {
                for (auto& kv : tasks_by_zone) {
                    if (kv.first == a_zone) continue;
                    for (int tid : kv.second) {
                        if (done_tasks.count(tid)) continue;
                        if (deferred_tasks.count(tid)) continue;
                        float bs = base_score(a_loc, tid, env);
                        if (bs > best_score) { best_score = bs; best_tid = tid; }
                    }
                }
            }

            // Deferred fallback
            if (best_tid < 0 && !deferred_tasks.empty()) {
                for (int tid : deferred_tasks) {
                    if (done_tasks.count(tid)) continue;
                    float bs = base_score(a_loc, tid, env);
                    if (bs > best_score) { best_score = bs; best_tid = tid; }
                }
            }

            if (best_tid >= 0) {
                proposed_schedule[a] = best_tid;
                // Per-assignment future flow update + instant lock
                {
                    auto tsk = env->task_pool.find(best_tid);
                    if (tsk != env->task_pool.end() && !tsk->second.locations.empty()) {
                        add_agent_future_flow(a_loc, tsk->second.locations, env, true);
                    }
                }

                auto tit = env->task_pool.find(best_tid);
                if (tit != env->task_pool.end())
                    g_parked_task[a] = (tit->second.locations.size() > 2) ? best_tid : -1;
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

    // Fallback: ensure all free agents have proposed_schedule set
    for (int a : free_agents)
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;

#if USE_DIAG_LOG
    // Timeout detection — write to CSV (reliable, like CorridorLog)
    static FILE* g_diag_fd = nullptr;
    if (!g_diag_fd) {
        g_diag_fd = fopen("C:/Users/Administrator/.openclaw/workspace/schedule_diag.csv", "w");
        if (g_diag_fd) {
            fprintf(g_diag_fd, "step,timeout_n,free_agents,phase2_assigned,busy,free_tasks,lock_r,lock_l,lock_0,narrow_lock,wide_total,widelock_lock\n");
            fflush(g_diag_fd);
        }
    }
    static int timeout_counter = 0;
    if (std::chrono::steady_clock::now() > endtime) {
        timeout_counter++;
        if (g_diag_fd) {
            fprintf(g_diag_fd, "%d,%d,%zu,%d,,,,,,,,,\n",
                    step, timeout_counter, free_agents.size(), phase2_assigned);
            fflush(g_diag_fd);
        }
    }
#endif

#if USE_DIAG_LOG
    // Log assignment stats
    CorridorLog::log_assign_stats(step, g_dbg_exempt, g_dbg_branch, g_dbg_dirlock, 0, g_dbg_accepted);
    g_dbg_exempt = 0; g_dbg_branch = 0; g_dbg_dirlock = 0; g_dbg_accepted = 0;
#endif

#if USE_DIAG_LOG
    // DIAG every 500 ticks
    if (env->curr_timestep > 0 && env->curr_timestep % 500 == 0) {
        int lock_r = 0, lock_l = 0, lock_0 = 0;
        int locked_narrow = 0, locked_wide = 0, wide_total = 0;
        for (int si = 0; si < Centerline::g_mp_count; si++) {
            int ld = Centerline::g_mp_lock_dir[si];
            if (ld > 0) lock_r++; else if (ld < 0) lock_l++; else lock_0++;
            int w = Centerline::g_mp_width_p10[si];
            if (w >= WIDTH_BAND_3) { wide_total++; if (ld != 0) locked_wide++; }
            else if (w < WIDTH_BAND_2) { if (ld != 0) locked_narrow++; }
        }
        int busy = 0;
        for (int a = 0; a < env->num_of_agents; a++)
            if (env->curr_task_schedule[a] >= 0) busy++;
        if (g_diag_fd) {
            fprintf(g_diag_fd, "%d,,,,%d,%zu,%d,%d,%d,%d,%d,%d,%d\n",
                   step, busy, free_tasks.size(),
                   lock_r, lock_l, lock_0, locked_narrow, wide_total, locked_wide);
            fflush(g_diag_fd);
        }
    }
#endif
}

} // namespace strategy_centerline
