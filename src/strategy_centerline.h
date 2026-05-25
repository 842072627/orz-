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

    // Exempt: fast path for trajectories that don't cross any narrow segment
    #if USE_TRAFFIC_PREDICTION
    {
        int a_m, a_b;
        Centerline::project(a_loc, a_m, a_b);
        if (a_m >= 0) {
            bool passes_narrow = false;
            int prev_m = a_m;
            for (auto& loc : locs) {
                int tm, tb;
                Centerline::project(loc, tm, tb);
                if (tm < 0) continue;
                int lo = std::min(prev_m, tm);
                int hi = std::max(prev_m, tm);
                for (auto& seg : Centerline::g_segments) {
                    if (!seg.is_narrow) continue;
                    int sm_lo = Centerline::g_mp_main_dist[seg.si_lo];
                    int sm_hi = Centerline::g_mp_main_dist[std::min(seg.si_hi, Centerline::g_mp_count) - 1];
                    if (hi > sm_lo && lo < sm_hi) { passes_narrow = true; break; }
                }
                if (passes_narrow) break;
                prev_m = tm;
            }
            if (!passes_narrow) {
                float total = 0;
                int prev = a_loc;
                for (auto& loc : locs) {
                    int d = unified_dist(prev, loc);
                    total += 1.0f / (float)(d + 1);
                    prev = loc;
                }
                g_dbg_exempt++;
                return total;
            }
        }
    }
    #endif

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
            float total = 0;
            int prev = a_loc;
            for (size_t ei = (size_t)start_idx; ei < locs.size(); ei++) {
                int loc = locs[ei];
                int d = unified_dist(prev, loc);
                total += 1.0f / (float)(d + 1);
                prev = loc;
            }
            g_dbg_branch++;
            return total;
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

        int d_max_seg = 0;
        auto lo_it = std::lower_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), lo);
        auto hi_it = std::upper_bound(Centerline::g_mp_main_dist.begin(), Centerline::g_mp_main_dist.end(), hi);
        for (int si = (int)(lo_it - Centerline::g_mp_main_dist.begin());
             si < (int)(hi_it - Centerline::g_mp_main_dist.begin()); si++) {
            int d = Centerline::g_mp_density[si];
            if (USE_DENSITY_HARD_CAP && d >= DENSITY_HARD_CAP)
                return -1e9f;
            if (d > d_max_seg) d_max_seg = d;
        }
        
        // Traffic prediction segment check
        #if USE_TRAFFIC_PREDICTION
        for (auto& seg : Centerline::g_segments) {
            if (!seg.is_narrow) continue;
            int sm_lo = Centerline::g_mp_main_dist[seg.si_lo];
            int sm_hi = Centerline::g_mp_main_dist[std::min(seg.si_hi, Centerline::g_mp_count - 1)];
            if (hi <= sm_lo || lo >= sm_hi) continue;
            
            if (seg.flow_dir != 0 && intent != 0 && intent != seg.flow_dir) {
                // Agent direction doesn't match segment flow
                // Allow if agent is already inside or in queue zone
                int a_m_here, a_b_here;
                Centerline::project(a_loc, a_m_here, a_b_here);
                bool in_queue = false;
                if (intent > 0 && a_m_here < sm_lo && sm_lo - a_m_here < 80)
                    in_queue = true;
                if (intent < 0 && a_m_here > sm_hi && a_m_here - sm_hi < 80)
                    in_queue = true;
                bool inside = (a_m_here >= sm_lo && a_m_here <= sm_hi);
                if (!in_queue && !inside) {
                    g_dbg_dirlock++;
                    return -1e9f;
                }
            }
        }
        #endif

        float seg_dist = (float)(std::abs(prev_m - tm) + std::abs(prev_b - tb));
        float seg_cost = (ei > (size_t)start_idx) ? (seg_dist * 0.3f + 1.0f) : (seg_dist + 1.0f);
        float seg_score = 1.0f / seg_cost;
        if (seg_cost > MAX_TASK_SEGMENT_DIST) {
            float penalty = (float)MAX_TASK_SEGMENT_DIST / seg_cost;
            seg_score *= penalty * penalty;
        }
        if (d_max_seg > ROUTE_THRESHOLD) {
            int idx = (d_max_seg > 40) ? 40 : d_max_seg;
            seg_score /= DIVISOR_TABLE[idx - ROUTE_THRESHOLD];
        }
        total_score += seg_score;
        prev_m = tm; prev_b = tb;
    }
    return total_score;
}

// ====== Narrow segment direction evaluation ======
#if USE_TRAFFIC_PREDICTION
static inline void evaluate_narrow_segments(SharedEnvironment* env) {
    for (auto& seg : Centerline::g_segments) {
        if (!seg.is_narrow) continue;
        
        // Direction based on predicted traffic
        int r = seg.right_count;
        int l = seg.left_count;
        
        if (r > l * 1.5f) seg.flow_dir = +1;
        else if (l > r * 1.5f) seg.flow_dir = -1;
        else if (seg.is_narrow && r + l > 0) {
            // Narrow with close counts: must pick one to prevent both-way
            seg.flow_dir = (r >= l) ? +1 : -1;
        }
        
        // Blockage detection
        if (seg.consecutive_no_exit >= BLOCK_THRESHOLD)
            seg.is_blocked = true;
        else
            seg.is_blocked = false;
    }
    
    // Group enforcement: same group = same flow_dir
    // Collect groups, pick majority direction, apply to all
    std::unordered_map<int, std::vector<int>> group_dirs;
    for (auto& seg : Centerline::g_segments) {
        if (!seg.is_narrow || seg.group_id < 0) continue;
        group_dirs[seg.group_id].push_back(seg.flow_dir);
    }
    for (auto& kv : group_dirs) {
        int gid = kv.first;
        auto& dirs = kv.second;
        if (dirs.empty()) continue;
        int sum = 0;
        for (int d : dirs) sum += d;
        int majority = (sum >= 0) ? +1 : -1;
        // Apply majority to all segments in this group
        for (auto& seg : Centerline::g_segments) {
            if (!seg.is_narrow || seg.group_id != gid) continue;
            seg.flow_dir = majority;
        }
    }
}
#endif

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

    // Agents with tasks: predict their trajectories
    for (auto& seg : Centerline::g_segments) {
        seg.right_count = seg.left_count = 0;
        seg.right_early = seg.left_early = 999999;
        seg.right_late = seg.left_late = 0;
        seg.entrance_queue_right = seg.entrance_queue_left = 0;
        seg.inside_count = 0;
        seg.inside_right = 0;
        seg.inside_left = 0;
        seg.consecutive_no_exit = 0;
        seg.is_blocked = false;
    }
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;  // idle agents: no prediction
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        Centerline::predict_trajectory(a, a_loc, it->second.locations, env);
    }
    
    // Track agent positions relative to narrow segments
    for (int a = 0; a < env->num_of_agents; a++) {
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        int a_m, a_b;
        Centerline::project(a_loc, a_m, a_b);
        if (a_m < 0) continue;
        int a_tid = env->curr_task_schedule[a];
        int a_target_m = -1;
        if (a_tid >= 0) {
            auto tit = env->task_pool.find(a_tid);
            if (tit != env->task_pool.end() && !tit->second.locations.empty()) {
                int t1m, t1b;
                Centerline::project(tit->second.locations[0], t1m, t1b);
                a_target_m = t1m;
            }
        }
        for (auto& seg : Centerline::g_segments) {
            int sm_lo = Centerline::g_mp_main_dist[seg.si_lo];
            int sm_hi = Centerline::g_mp_main_dist[std::min(seg.si_hi, Centerline::g_mp_count) - 1];
            if (a_m >= sm_lo && a_m <= sm_hi) {
                // Agent is inside this segment
                seg.inside_count++;
                if (a_target_m >= 0) {
                    int dir = (a_target_m > a_m) ? +1 : -1;
                    if (dir > 0) seg.inside_right++;
                    else seg.inside_left++;
                }
            } else if (a_m < sm_lo && sm_lo - a_m < 80) {
                seg.entrance_queue_right++;
            } else if (a_m > sm_hi && a_m - sm_hi < 80) {
                seg.entrance_queue_left++;
            }
        }
    }
    
    Centerline::g_current_tick++;
    
    // Evaluate narrow segment direction
    evaluate_narrow_segments(env);
    
    CenterlineAccess::g_branch_dist   = &Centerline::g_branch_dist;
    CenterlineAccess::g_main_dist     = &Centerline::g_main_dist;
    CenterlineAccess::g_zone_is_main  = &Centerline::g_zone_is_main;
    CenterlineAccess::g_mp_loc        = &Centerline::g_mp_loc;
    CenterlineAccess::g_mp_main_dist  = &Centerline::g_mp_main_dist;
    CenterlineAccess::g_mp_count      = Centerline::g_mp_count;
    CenterlineAccess::g_n24c          = Centerline::g_n24c;
    CenterlineAccess::g_n24r          = Centerline::g_n24r;
#if USE_TRAFFIC_PREDICTION
    CenterlineAccess::g_segments_ptr  = &Centerline::g_segments;
#endif

#if USE_DIAG_LOG
    CorridorLog::log_tick(env);
    // Only write debug CSV every 10 ticks to keep performance
    if (env->curr_timestep % 10 == 0)
        Centerline::debug_traffic_csv();
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
                // Per-assignment: add trajectory to prediction table
                #if USE_TRAFFIC_PREDICTION
                {
                    auto tsk = env->task_pool.find(best_tid);
                    if (tsk != env->task_pool.end() && !tsk->second.locations.empty()) {
                        Centerline::predict_trajectory(a, a_loc, tsk->second.locations, env);
                    }
                }
                #endif

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

}

} // namespace strategy_centerline
