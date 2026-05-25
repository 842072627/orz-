// strategy_centerline.h — V4: heat-based direction lock + narrow group control
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
#if USE_DIAG_LOG
    printf("CENTERLINE_SCHEDULER: agents=%d, map=%dx%d, zones=%zu, cells=%zu\n",
           env->num_of_agents, env->cols, env->rows,
           Centerline::g_zone_ranges.size(), Centerline::g_centerline_cells.size());
    CorridorLog::init();
    CorridorLog::set_env(env);
#endif
}

// ====== base_score ======
static inline float base_score(int a_loc, int tid, SharedEnvironment* env, int start_idx = 0) {
    auto it = env->task_pool.find(tid);
    if (it == env->task_pool.end() || it->second.locations.empty())
        return -1e9f;
    if (!Centerline::g_ready) return 1.0f;
    auto& locs = it->second.locations;
    
    int a_m, a_b;
    Centerline::project(a_loc, a_m, a_b);
    
    // Step 1: compute task1→task2 direction and distance
    int t1_m = -1, t2_m = -1;
    if (!locs.empty()) {
        int tb;
        Centerline::project(locs[0], t1_m, tb);
    }
    if (locs.size() > 1) {
        int tb;
        Centerline::project(locs[1], t2_m, tb);
    }
    
    int task_dir = 0;
    int task_dist = 0;
    if (t1_m >= 0 && t2_m >= 0) {
        task_dir = (t2_m > t1_m) ? +1 : (t2_m < t1_m) ? -1 : 0;
        task_dist = std::abs(t2_m - t1_m);
    }
    
    // Step 2: direction lock check on all narrow groups that task1→task2 passes
    bool dir_blocked = false;
    if (task_dir != 0 && t1_m >= 0 && t2_m >= 0) {
        int lo = std::min(t1_m, t2_m);
        int hi = std::max(t1_m, t2_m);
        for (auto& seg : Centerline::g_segments) {
            if (!seg.is_narrow) continue;
            if (seg.flow_dir == 0) continue;  // unlocked
            int sm_lo = Centerline::g_mp_main_dist[seg.si_lo];
            int sm_hi = Centerline::g_mp_main_dist[std::min(seg.si_hi, Centerline::g_mp_count) - 1];
            if (hi <= sm_lo || lo >= sm_hi) continue;
            if (task_dir != seg.flow_dir) {
                dir_blocked = true;
                break;
            }
        }
    }
    if (dir_blocked) {
        g_dbg_dirlock++;
        return -1e9f;
    }
    
    // Step 3: agent→task1 heat check (only if direction mismatches flow and flow is locked)
    if (a_m >= 0 && t1_m >= 0) {
        int agent_dir = (t1_m > a_m) ? +1 : (t1_m < a_m) ? -1 : 0;
        bool check_heat = false;
        // Only check heat if agent_dir mismatches some group's flow_dir
        int alo = std::min(a_m, t1_m);
        int ahi = std::max(a_m, t1_m);
        for (auto& seg : Centerline::g_segments) {
            if (!seg.is_narrow || seg.flow_dir == 0) continue;
            int sm_lo = Centerline::g_mp_main_dist[seg.si_lo];
            int sm_hi = Centerline::g_mp_main_dist[std::min(seg.si_hi, Centerline::g_mp_count) - 1];
            if (ahi <= sm_lo || alo >= sm_hi) continue;
            if (agent_dir != seg.flow_dir) {
                check_heat = true;
                break;
            }
        }
        if (check_heat) {
            for (int si = 0; si < Centerline::g_mp_count; si++) {
                int sm = Centerline::g_mp_main_dist[si];
                if (sm >= alo && sm <= ahi) {
                    if (Centerline::g_mp_density[si] >= HEAT_AGENT_TASK_REJECT) {
                        return -1e9f;
                    }
                }
            }
        }
    }
    
    // Step 4: distance score (agent→task1)
    float d = (float)unified_dist(a_loc, locs[0]);
    return -(d + 1.0f);  // negative = shorter is better (sorted separately by task_dist)
}

// ====== evaluate narrow segments (heat-based lock/unlock) ======
#if USE_TRAFFIC_PREDICTION
static inline void evaluate_narrow_segments(SharedEnvironment* env) {
    // Step 1: per-segment raw direction from prediction
    for (auto& seg : Centerline::g_segments) {
        if (!seg.is_narrow) continue;
        int r = seg.right_count, l = seg.left_count;
        if (r > l) seg.flow_dir = +1;
        else if (l > r) seg.flow_dir = -1;
        else seg.flow_dir = (r + l > 0) ? ((rand()%2==0)?+1:-1) : seg.flow_dir;
    }
    
    // Step 2: collect per-group stats
    std::unordered_map<int, std::pair<int,int>> group_flow;
    std::unordered_map<int, int> group_max_heat;
    for (auto& seg : Centerline::g_segments) {
        if (!seg.is_narrow || seg.group_id < 0) continue;
        int gid = seg.group_id;
        group_flow[gid].first += seg.right_count;
        group_flow[gid].second += seg.left_count;
        for (int si = seg.si_lo; si < seg.si_hi && si < Centerline::g_mp_count; si++) {
            int h = Centerline::g_mp_density[si];
            if (h > group_max_heat[gid]) group_max_heat[gid] = h;
        }
    }
    
    // Step 3: heat-based lock/unlock, group enforcement
    for (auto& seg : Centerline::g_segments) {
        if (!seg.is_narrow || seg.group_id < 0) continue;
        int gid = seg.group_id;
        int max_heat = group_max_heat[gid];
        if (max_heat < HEAT_UNLOCK_THRESHOLD) {
            seg.flow_dir = 0;  // unlock
        } else {
            int sum_r = group_flow[gid].first;
            int sum_l = group_flow[gid].second;
            seg.flow_dir = (sum_r >= sum_l) ? +1 : -1;
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

    // Clear and predict agent trajectories
    for (auto& seg : Centerline::g_segments) {
        seg.right_count = seg.left_count = 0;
        seg.right_early = seg.left_early = 999999;
        seg.right_late = seg.left_late = 0;
        seg.entrance_queue_right = seg.entrance_queue_left = 0;
        seg.inside_count = 0;
        seg.inside_right = seg.inside_left = 0;
        seg.consecutive_no_exit = 0;
        seg.is_blocked = false;
    }
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int tid = env->curr_task_schedule[a];
        if (tid < 0) continue;
        auto it = env->task_pool.find(tid);
        if (it == env->task_pool.end() || it->second.locations.empty()) continue;
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        Centerline::predict_trajectory(a, a_loc, it->second.locations, env);
    }
    
    // Track agent positions
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

    // Phase 1: Continue multi-errand tasks (same as before)
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

    // Phase 2: Greedy assignment with task1→task2 distance ordering
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

        // Garbage defer (same as before)
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

        // Sort free_tasks by task1→task2 distance (shorter first)
        // This is done per-agent in the loop below using the score
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

            // Collect all candidate tasks for this agent
            struct TaskCand {
                int tid;
                int task_dist;   // task1→task2
                float agent_dist; // agent→task1
            };
            std::vector<TaskCand> candidates;
            
            // Same zone
            auto sz = tasks_by_zone.find(a_zone);
            if (sz != tasks_by_zone.end()) {
                for (int tid : sz->second) {
                    if (done_tasks.count(tid) || deferred_tasks.count(tid)) continue;
                    auto it = env->task_pool.find(tid);
                    if (it == env->task_pool.end() || it->second.locations.size() < 2) continue;
                    int m1, b1, m2, b2;
                    Centerline::project(it->second.locations[0], m1, b1);
                    Centerline::project(it->second.locations[1], m2, b2);
                    if (m1 < 0 || m2 < 0) continue;
                    int td = std::abs(m2 - m1);
                    float ad = (float)unified_dist(a_loc, it->second.locations[0]);
                    candidates.push_back({tid, td, ad});
                }
            }
            // Other zones
            for (auto& kv : tasks_by_zone) {
                if (kv.first == a_zone) continue;
                for (int tid : kv.second) {
                    if (done_tasks.count(tid)) continue;
                    if (deferred_tasks.count(tid)) continue;
                    auto it = env->task_pool.find(tid);
                    if (it == env->task_pool.end() || it->second.locations.size() < 2) continue;
                    int m1, b1, m2, b2;
                    Centerline::project(it->second.locations[0], m1, b1);
                    Centerline::project(it->second.locations[1], m2, b2);
                    if (m1 < 0 || m2 < 0) continue;
                    int td = std::abs(m2 - m1);
                    float ad = (float)unified_dist(a_loc, it->second.locations[0]);
                    candidates.push_back({tid, td, ad});
                }
            }
            // Deferred fallback
            for (int tid : deferred_tasks) {
                if (done_tasks.count(tid)) continue;
                auto it = env->task_pool.find(tid);
                if (it == env->task_pool.end() || it->second.locations.size() < 2) continue;
                int m1, b1, m2, b2;
                Centerline::project(it->second.locations[0], m1, b1);
                Centerline::project(it->second.locations[1], m2, b2);
                if (m1 < 0 || m2 < 0) continue;
                int td = std::abs(m2 - m1);
                float ad = (float)unified_dist(a_loc, it->second.locations[0]);
                candidates.push_back({tid, td, ad});
            }
            
            // Sort by task_dist first, then agent_dist
            std::sort(candidates.begin(), candidates.end(),
                [&](const TaskCand& a, const TaskCand& b) {
                    if (a.task_dist != b.task_dist) return a.task_dist < b.task_dist;
                    return a.agent_dist < b.agent_dist;
                });
            
            // Score in sorted order (shorter tasks first)
            for (auto& cand : candidates) {
                float bs = base_score(a_loc, cand.tid, env);
                if (bs > best_score) { best_score = bs; best_tid = cand.tid; }
            }

            if (best_tid >= 0) {
                proposed_schedule[a] = best_tid;
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

    // Fallback
    for (int a : free_agents)
        if (proposed_schedule[a] < 0) proposed_schedule[a] = -1;
}

} // namespace strategy_centerline
