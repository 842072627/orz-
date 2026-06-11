// optsvr/schedule/IronZoneSchedule/IronZoneScheduler.cpp
// IRON mode 10: coarse service-zone assignment with compact diagnostics.
#include "IronZoneScheduler.h"
#include "HeuristicMatrix.h"
#include "RuntimeConfig.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <numeric>
#include <queue>
#include <thread>
#include <tuple>
#include <unordered_set>

namespace optsvr {

namespace {

constexpr int H_CAP = 100000;

double elapsed_ms(std::chrono::steady_clock::time_point a,
                  std::chrono::steady_clock::time_point b) {
    return std::chrono::duration_cast<std::chrono::microseconds>(b - a).count() / 1000.0;
}

int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} // namespace

void IronZoneScheduler::initialize(SharedEnvironment* env, int /*preprocess_time_limit_ms*/) {
    env_ = env;
    rows_ = env ? env->rows : 0;
    cols_ = env ? env->cols : 0;
    map_size_ = rows_ * cols_;

    zone_rows_ = std::max(1, cfg().iron_zone_rows);
    zone_cols_ = std::max(1, cfg().iron_zone_cols);
    zone_count_ = zone_rows_ * zone_cols_;
    build_components(env);

    std::fprintf(stderr,
        "[IronZone] init zones=%dx%d comps=%d variant=%d radius=%d..%d topk=%d weights(dist=%.2f,len=%.2f,cross=%.2f,end=%.2f,age=%.2f,balance=%.2f,migrate=%.2f) early(ticks=%d,dist=%.2f,cross=%.2f,long=%.2f) pool(short=%d,starved=%d,th=%d) aggressive(on=%d,iters=%d,probe=%d,margin=%.1f) opp(ratio=%.3f,dist=%d,comp=%d,cross=%d) assign_ratio=%.2f\n",
        zone_rows_, zone_cols_,
        component_count_,
        cfg().iron_assign_variant,
        cfg().iron_local_radius, cfg().iron_max_radius,
        cfg().iron_candidate_topk,
        cfg().iron_dist_weight, cfg().iron_task_len_weight,
        cfg().iron_cross_zone_w, cfg().iron_endpoint_w,
        cfg().iron_age_weight, cfg().iron_balance_w, cfg().iron_migration_w,
        cfg().iron_early_phase_ticks, cfg().iron_early_dist_w,
        cfg().iron_early_cross_w, cfg().iron_early_long_w,
        cfg().iron_short_task_topk, cfg().iron_starved_task_topk,
        cfg().iron_starved_zone_threshold,
        cfg().iron_aggressive_assign ? 1 : 0,
        cfg().iron_aggressive_iters,
        cfg().iron_aggressive_probe,
        cfg().iron_aggressive_replace_margin,
        cfg().iron_opportunity_ratio,
        cfg().iron_opportunity_dist,
        cfg().iron_opportunity_completion,
        cfg().iron_opportunity_cross,
        cfg().iron_assign_ratio);
}

int IronZoneScheduler::zone_of_loc(int loc) const {
    if (loc < 0 || loc >= map_size_ || cols_ <= 0) return -1;
    int r = loc / cols_;
    int c = loc % cols_;
    int zr = std::min(zone_rows_ - 1,
                      static_cast<int>((long long)r * zone_rows_ / std::max(1, rows_)));
    int zc = std::min(zone_cols_ - 1,
                      static_cast<int>((long long)c * zone_cols_ / std::max(1, cols_)));
    return zr * zone_cols_ + zc;
}

int IronZoneScheduler::component_of_loc(int loc) const {
    if (loc < 0 || loc >= static_cast<int>(component_id_.size())) return -1;
    return component_id_[loc];
}

void IronZoneScheduler::build_components(SharedEnvironment* env) {
    component_count_ = 0;
    component_id_.assign(map_size_, -1);
    if (!env || map_size_ <= 0 || cols_ <= 0) return;

    std::deque<int> q;
    for (int loc = 0; loc < map_size_; loc++) {
        if (env->map[loc] != 0 || component_id_[loc] != -1) continue;

        const int cid = component_count_++;
        component_id_[loc] = cid;
        q.push_back(loc);

        while (!q.empty()) {
            int cur = q.front();
            q.pop_front();
            int r = cur / cols_;
            int c = cur % cols_;

            auto push_if_free = [&](int next) {
                if (next < 0 || next >= map_size_) return;
                if (env->map[next] != 0 || component_id_[next] != -1) return;
                component_id_[next] = cid;
                q.push_back(next);
            };

            if (r > 0) push_if_free(cur - cols_);
            if (r + 1 < rows_) push_if_free(cur + cols_);
            if (c > 0) push_if_free(cur - 1);
            if (c + 1 < cols_) push_if_free(cur + 1);
        }
    }
}

bool IronZoneScheduler::task_remaining_single_component(
    const Task& task, int start_idx, int* component) const
{
    if (component) *component = -1;
    if (start_idx < 0 || start_idx >= static_cast<int>(task.locations.size())) {
        return false;
    }

    int cid = -1;
    for (int i = start_idx; i < static_cast<int>(task.locations.size()); i++) {
        int loc = task.locations[i];
        int loc_component = component_of_loc(loc);
        if (loc_component < 0) return false;
        if (cid < 0) {
            cid = loc_component;
        } else if (loc_component != cid) {
            return false;
        }
    }
    if (component) *component = cid;
    return cid >= 0;
}

int IronZoneScheduler::zone_distance(int za, int zb) const {
    if (za < 0 || zb < 0) return zone_rows_ + zone_cols_;
    int ar = za / zone_cols_;
    int ac = za % zone_cols_;
    int br = zb / zone_cols_;
    int bc = zb % zone_cols_;
    return std::abs(ar - br) + std::abs(ac - bc);
}

int IronZoneScheduler::distance_fast(SharedEnvironment* env, int source, int target) const {
    if (!env || source < 0 || target < 0 ||
        source >= map_size_ || target >= map_size_) {
        return H_CAP;
    }
    if (!heuristic_matrix().is_ready()) {
        int sr = source / cols_;
        int sc = source % cols_;
        int tr = target / cols_;
        int tc = target % cols_;
        return std::abs(sr - tr) + std::abs(sc - tc);
    }
    int h = get_h_fast(env, source, target);
    return h >= H_CAP ? H_CAP : h;
}

int IronZoneScheduler::effective_tick(SharedEnvironment* env) const {
    if (!env) return 0;
    if (env->system_timestep >= 0) return env->system_timestep;
    return env->curr_timestep;
}

float IronZoneScheduler::early_phase_factor() const {
    int phase_ticks = cfg().iron_early_phase_ticks;
    if (!env_ || phase_ticks <= 0) return 0.0f;
    int tick = effective_tick(env_);
    if (tick >= phase_ticks) return 0.0f;
    int remaining = phase_ticks - tick;
    return static_cast<float>(remaining) / static_cast<float>(phase_ticks);
}

int IronZoneScheduler::first_remaining_index(const Task& task) const {
    if (task.locations.empty()) return -1;
    int idx = task.idx_next_loc;
    if (idx < 0 || idx >= static_cast<int>(task.locations.size())) idx = 0;
    return idx;
}

int IronZoneScheduler::remaining_task_length(SharedEnvironment* env, const Task& task) const {
    int idx = first_remaining_index(task);
    if (idx < 0) return 0;
    int length = 0;
    int prev = task.locations[idx];
    for (int i = idx + 1; i < static_cast<int>(task.locations.size()); i++) {
        int next = task.locations[i];
        length += distance_fast(env, prev, next);
        prev = next;
    }
    return length;
}

float IronZoneScheduler::score_candidate(
    const AgentMeta& agent,
    const TaskMeta& task,
    const std::vector<int>& finish_zone_pressure,
    const std::vector<int>& zone_shortage) const
{
    int dist = distance_fast(env_, agent.loc, task.start);

    int zd = zone_distance(agent.zone, task.start_zone);
    float early = early_phase_factor();
    if (cfg().iron_assign_variant == 14 ||
        cfg().iron_assign_variant == 16) {
        early = 1.0f;
    }
    float dist_w = cfg().iron_dist_weight + early * cfg().iron_early_dist_w;
    float len_w = cfg().iron_task_len_weight + early * cfg().iron_early_long_w;
    float cross_w = cfg().iron_cross_zone_w + early * cfg().iron_early_cross_w;
    if (cfg().iron_assign_variant == 6) {
        dist_w = 7.5f + 1.5f * early;
        len_w = 0.45f + 0.15f * (1.0f - early);
        cross_w = 110.0f + 30.0f * early;
    } else if (cfg().iron_assign_variant == 8 ||
               cfg().iron_assign_variant == 9 ||
               cfg().iron_assign_variant == 10 ||
               cfg().iron_assign_variant == 11) {
        dist_w = 5.5f + 1.0f * early;
        len_w = 0.90f + 0.20f * early;
        cross_w = 100.0f + 20.0f * early;
    } else if (cfg().iron_assign_variant == 12) {
        dist_w = 7.0f + 1.5f * early;
        len_w = 0.70f + 0.15f * early;
        cross_w = 115.0f + 25.0f * early;
    } else if (cfg().iron_assign_variant == 15) {
        dist_w = 4.0f + 0.5f * early;
        len_w = 1.35f + 0.15f * early;
        cross_w = 85.0f + 15.0f * early;
    }
    float score = dist_w * static_cast<float>(dist)
                + len_w * static_cast<float>(task.length)
                + cross_w * static_cast<float>(zd)
                - cfg().iron_age_weight * static_cast<float>(task.age);

    if (!zone_shortage.empty() &&
        task.start_zone >= 0 &&
        task.start_zone < static_cast<int>(zone_shortage.size())) {
        int start_shortage = zone_shortage[task.start_zone];
        score -= cfg().iron_balance_w * static_cast<float>(start_shortage);

        if (agent.zone >= 0 &&
            agent.zone < static_cast<int>(zone_shortage.size()) &&
            task.finish_zone >= 0 &&
            task.finish_zone < static_cast<int>(zone_shortage.size())) {
            int migration_gain = zone_shortage[task.finish_zone] - zone_shortage[agent.zone];
            score -= cfg().iron_migration_w * static_cast<float>(migration_gain);
        }
    }

    if (cfg().iron_endpoint_w != 0.0f &&
        task.finish_zone >= 0 &&
        task.finish_zone < static_cast<int>(finish_zone_pressure.size())) {
        score += cfg().iron_endpoint_w *
                 static_cast<float>(finish_zone_pressure[task.finish_zone]);
    }
    if (cfg().iron_assign_variant == 3 &&
        task.finish_zone >= 0 &&
        task.finish_zone < static_cast<int>(finish_zone_pressure.size())) {
        int avg_pressure = zone_count_ > 0
            ? std::max(1, env_->num_of_agents / zone_count_)
            : 1;
        int finish_excess = std::max(0, finish_zone_pressure[task.finish_zone] - avg_pressure);
        score += 10.0f * static_cast<float>(finish_excess);
    }
    return score;
}

void IronZoneScheduler::print_round_stats(const RoundStats& s, int tick) const {
    double avg_dist = s.assigned > 0 ? static_cast<double>(s.dist_sum) / s.assigned : 0.0;
    double avg_len = s.assigned > 0 ? static_cast<double>(s.len_sum) / s.assigned : 0.0;
    double avg_zd = s.assigned > 0 ? static_cast<double>(s.zone_dist_sum) / s.assigned : 0.0;
    double avg_completion = s.assigned > 0
        ? static_cast<double>(s.completion_cost_sum) / s.assigned
        : 0.0;
    double avg_cand = s.candidate_agents > 0
        ? static_cast<double>(s.candidate_total) / s.candidate_agents
        : 0.0;
    double avg_start_shortage = s.assigned > 0
        ? static_cast<double>(s.start_shortage_sum) / s.assigned
        : 0.0;
    double avg_finish_shortage = s.assigned > 0
        ? static_cast<double>(s.finish_shortage_sum) / s.assigned
        : 0.0;
    double avg_migration_gain = s.assigned > 0
        ? static_cast<double>(s.migration_gain_sum) / s.assigned
        : 0.0;
    double cross_pct = s.assigned > 0 ? 100.0 * s.cross_zone / s.assigned : 0.0;
    double long_pct = s.assigned > 0 ? 100.0 * s.long_tasks / s.assigned : 0.0;
    int busy_after = s.busy_before + s.assigned;

    std::fprintf(stderr,
        "[IronZone] tick=%d phase=%.2f variant=%d target=%d(base=%d) freeA=%d freeT=%d assigned=%d busy=%d unmatched=%d opp(cap=%d,extra=%d,reject=%d) avgDist=%.1f avgLen=%.1f avgComp=%.1f cross=%.1f%% avgZ=%.2f shortage(start=%.1f,finish=%.1f,gain=%.1f,zones=%d) pool(short=%d,starved=%d) skip(comp=%d,empty=%d,quota=%d,gate=%d) freeLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignLen(<=500=%d,<=1000=%d,<=1500=%d,>1500=%d) assignDist(<=50=%d,<=150=%d,>150=%d) long>=%d=%.1f%% cand=%.1f opt(rounds=%d,seed=%d,replace=%d,swap=%d) heap(pops=%d,skip=%d,fb=%d,gfb=%d) time(build=%.1fms,solve=%.1fms)\n",
        tick, s.early_phase, s.assign_variant, s.target_active, s.base_target_active,
        s.free_agents, s.free_tasks, s.assigned, busy_after, s.unmatched_agents,
        s.opportunity_cap, s.opportunity_assigned, s.opportunity_rejected,
        avg_dist, avg_len, avg_completion, cross_pct, avg_zd,
        avg_start_shortage, avg_finish_shortage, avg_migration_gain,
        s.starved_zones, s.short_pool, s.starved_pool,
        s.skipped_component, s.candidate_empty_agents, s.quota_skips, s.gate_skips,
        s.free_len_short, s.free_len_mid, s.free_len_long, s.free_len_huge,
        s.assign_len_short, s.assign_len_mid, s.assign_len_long, s.assign_len_huge,
        s.assign_dist_near, s.assign_dist_mid, s.assign_dist_far,
        cfg().iron_long_task_th, long_pct, avg_cand,
        s.aggressive_rounds, s.aggressive_seeded,
        s.aggressive_replacements, s.aggressive_swaps,
        s.heap_pops, s.lazy_skips, s.fallback_assigned, s.global_pool_assigned,
        s.build_ms, s.solve_ms);
}

void IronZoneScheduler::schedule(int time_limit_ms,
                                 std::vector<int>& proposed_schedule,
                                 SharedEnvironment* env) {
    if (!env || env->num_of_agents <= 0) return;
    env_ = env;
    const int cur_tick = effective_tick(env);

    auto start_time = std::chrono::steady_clock::now();
    auto deadline = start_time + std::chrono::milliseconds(std::max(1, time_limit_ms));

    const int n_agents = env->num_of_agents;
    if (static_cast<int>(proposed_schedule.size()) < n_agents ||
        static_cast<int>(env->curr_states.size()) < n_agents) {
        return;
    }

    try {
        task_pool_snapshot_ = env->task_pool;
        std::vector<State> states_copy = env->curr_states;
        curr_locations_snapshot_.assign(n_agents, -1);
        for (int a = 0; a < n_agents && a < static_cast<int>(states_copy.size()); a++) {
            curr_locations_snapshot_[a] = states_copy[a].location;
        }
        schedule_snapshot_.assign(proposed_schedule.begin(), proposed_schedule.end());
    } catch (...) {
        std::fprintf(stderr, "[IronZone] snapshot copy failed, skipping schedule round\n");
        return;
    }

    RoundStats stats;
    stats.early_phase = early_phase_factor();
    stats.assign_variant = cfg().iron_assign_variant;
    stats.free_agents = 0;

    std::vector<int> zone_agent_supply(zone_count_, 0);
    for (int a = 0; a < n_agents; a++) {
        int z = zone_of_loc(curr_locations_snapshot_[a]);
        if (z >= 0 && z < zone_count_) zone_agent_supply[z]++;
    }
    std::vector<int> zone_free_agents(zone_count_, 0);

    std::vector<AgentMeta> free_agents;
    free_agents.reserve(n_agents);
    std::unordered_set<int> assigned_tasks;
    assigned_tasks.reserve(n_agents * 2 + 1);

    for (int a = 0; a < n_agents; a++) {
        int tid = schedule_snapshot_[a];
        if (tid < 0) {
            int loc = curr_locations_snapshot_[a];
            int z = zone_of_loc(loc);
            int comp = component_of_loc(loc);
            if (loc >= 0 && loc < map_size_ && z >= 0 && comp >= 0) {
                free_agents.push_back({a, loc, z, comp});
                zone_free_agents[z]++;
            }
        } else {
            assigned_tasks.insert(tid);
            stats.busy_before++;
        }
    }
    stats.free_agents = static_cast<int>(free_agents.size());

    std::vector<TaskMeta> tasks;
    tasks.reserve(task_pool_snapshot_.size());
    std::vector<std::vector<int>> tasks_by_zone(zone_count_);
    std::vector<int> zone_free_tasks(zone_count_, 0);
    std::vector<int> finish_zone_pressure = zone_agent_supply;

    for (const auto& [tid, task] : task_pool_snapshot_) {
        if (task.locations.empty()) continue;
        int idx = first_remaining_index(task);
        if (idx < 0) continue;
        int task_component = -1;
        if (!task_remaining_single_component(task, idx, &task_component)) {
            stats.skipped_component++;
            continue;
        }
        int finish = task.locations.back();
        int finish_zone = zone_of_loc(finish);

        if (assigned_tasks.find(tid) != assigned_tasks.end()) {
            if (finish_zone >= 0 && finish_zone < zone_count_) {
                finish_zone_pressure[finish_zone]++;
            }
            continue;
        }

        int start = task.locations[idx];
        int start_zone = zone_of_loc(start);
        if (start_zone < 0 || finish_zone < 0) continue;

        TaskMeta meta;
        meta.tid = tid;
        meta.start = start;
        meta.finish = finish;
        meta.start_zone = start_zone;
        meta.finish_zone = finish_zone;
        meta.component = task_component;
        meta.length = remaining_task_length(env, task);
        meta.age = std::max(0, cur_tick - task.t_revealed);
        if (meta.length <= 500) stats.free_len_short++;
        else if (meta.length <= 1000) stats.free_len_mid++;
        else if (meta.length <= 1500) stats.free_len_long++;
        else stats.free_len_huge++;

        int task_index = static_cast<int>(tasks.size());
        tasks.push_back(meta);
        tasks_by_zone[start_zone].push_back(task_index);
        zone_free_tasks[start_zone]++;
    }
    stats.free_tasks = static_cast<int>(tasks.size());

    std::vector<int> zone_shortage(zone_count_, 0);
    int shortage_cap = std::max(1, cfg().iron_balance_cap);
    int starved_threshold = std::max(1, cfg().iron_starved_zone_threshold);
    for (int z = 0; z < zone_count_; z++) {
        int raw_shortage = zone_free_tasks[z] - zone_free_agents[z];
        zone_shortage[z] = clamp_int(raw_shortage, -shortage_cap, shortage_cap);
        if (zone_shortage[z] >= starved_threshold) stats.starved_zones++;
    }

    if (free_agents.empty() || tasks.empty()) {
        int interval = cfg().iron_stats_interval;
        if (interval > 0 && cur_tick - last_stats_tick_ >= interval) {
            last_stats_tick_ = cur_tick;
            print_round_stats(stats, cur_tick);
        }
        return;
    }

    int max_total = n_agents;
    if (cfg().iron_assign_ratio < 0.999f) {
        max_total = std::max(1, static_cast<int>(cfg().iron_assign_ratio * n_agents));
    }
    if (cfg().iron_assign_variant == 1 && stats.early_phase > 0.0) {
        float progress = 1.0f - static_cast<float>(stats.early_phase);
        float target_ratio = 0.70f + 0.30f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 4) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 4500.0f));
        float target_ratio = 0.78f + 0.22f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 5) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 3000.0f));
        float target_ratio = 0.88f + 0.12f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 6) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 2200.0f));
        float target_ratio = 0.84f + 0.16f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 7) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 3500.0f));
        float target_ratio = 0.82f + 0.18f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 8) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 10000.0f));
        float target_ratio = 0.70f + 0.30f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 9) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.70f * n_agents)));
    } else if (cfg().iron_assign_variant == 10) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.75f * n_agents)));
    } else if (cfg().iron_assign_variant == 11) {
        float progress = std::min(1.0f, std::max(0.0f, cur_tick / 8000.0f));
        float target_ratio = 0.70f + 0.10f * progress;
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(target_ratio * n_agents)));
    } else if (cfg().iron_assign_variant == 12) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.80f * n_agents)));
    } else if (cfg().iron_assign_variant == 13 ||
               cfg().iron_assign_variant == 14 ||
               cfg().iron_assign_variant == 15 ||
               cfg().iron_assign_variant == 16) {
        max_total = std::min(max_total,
                             std::max(1, static_cast<int>(0.70f * n_agents)));
    }
    const int base_target_active = max_total;
    stats.base_target_active = base_target_active;

    int base_new_assign = std::min<int>(free_agents.size(), tasks.size());
    base_new_assign = std::min(base_new_assign,
                               std::max(0, base_target_active - stats.busy_before));

    int opportunity_slots = 0;
    if (cfg().iron_assign_variant == 16 && cfg().iron_opportunity_ratio > 0.0f) {
        stats.opportunity_cap = std::max(0,
            static_cast<int>(cfg().iron_opportunity_ratio * n_agents));
        const int allowed_total = std::min(n_agents,
                                           base_target_active + stats.opportunity_cap);
        const int active_after_base = stats.busy_before + base_new_assign;
        opportunity_slots = std::max(0, allowed_total - active_after_base);
        opportunity_slots = std::min<int>(opportunity_slots,
                                          free_agents.size() - base_new_assign);
        opportunity_slots = std::min<int>(opportunity_slots,
                                          tasks.size() - base_new_assign);
        stats.target_active = allowed_total;
    } else {
        stats.target_active = base_target_active;
    }

    int max_new_assign = base_new_assign + opportunity_slots;
    if (max_new_assign <= 0) return;

    const int topk = std::max(1, cfg().iron_candidate_topk);
    const int local_radius = std::max(0, cfg().iron_local_radius);
    const int max_radius = std::max(local_radius, cfg().iron_max_radius);
    const int target_pool = std::max(topk * 2, topk);
    const bool aggressive_assign =
        cfg().iron_aggressive_assign && cfg().iron_aggressive_iters > 0;

    auto candidate_score = [&](const AgentMeta& agent, const TaskMeta& task) {
        return score_candidate(agent, task, finish_zone_pressure, zone_shortage);
    };

    auto optimization_score = [&](const AgentMeta& agent, const TaskMeta& task) {
        return score_candidate(agent, task, finish_zone_pressure, zone_shortage);
    };

    std::vector<int> short_task_pool;
    int short_topk = std::max(0, cfg().iron_short_task_topk);
    if (short_topk > 0) {
        short_task_pool.resize(tasks.size());
        std::iota(short_task_pool.begin(), short_task_pool.end(), 0);
        auto by_short_task = [&](int lhs, int rhs) {
            if (tasks[lhs].length != tasks[rhs].length) {
                return tasks[lhs].length < tasks[rhs].length;
            }
            return tasks[lhs].tid < tasks[rhs].tid;
        };
        if (static_cast<int>(short_task_pool.size()) > short_topk) {
            std::nth_element(short_task_pool.begin(),
                             short_task_pool.begin() + short_topk,
                             short_task_pool.end(),
                             by_short_task);
            short_task_pool.resize(short_topk);
        }
        std::sort(short_task_pool.begin(), short_task_pool.end(), by_short_task);
    }
    stats.short_pool = static_cast<int>(short_task_pool.size());

    std::vector<int> starved_task_pool;
    int starved_topk = std::max(0, cfg().iron_starved_task_topk);
    if (starved_topk > 0) {
        starved_task_pool.reserve(std::min<int>(tasks.size(), starved_topk));
        for (int ti = 0; ti < static_cast<int>(tasks.size()); ti++) {
            int z = tasks[ti].start_zone;
            if (z >= 0 &&
                z < static_cast<int>(zone_shortage.size()) &&
                zone_shortage[z] >= starved_threshold) {
                starved_task_pool.push_back(ti);
            }
        }
        auto by_starved_task = [&](int lhs, int rhs) {
            int lz = tasks[lhs].start_zone;
            int rz = tasks[rhs].start_zone;
            int ls = (lz >= 0 && lz < static_cast<int>(zone_shortage.size())) ? zone_shortage[lz] : 0;
            int rs = (rz >= 0 && rz < static_cast<int>(zone_shortage.size())) ? zone_shortage[rz] : 0;
            if (ls != rs) return ls > rs;
            if (tasks[lhs].length != tasks[rhs].length) {
                return tasks[lhs].length < tasks[rhs].length;
            }
            return tasks[lhs].tid < tasks[rhs].tid;
        };
        if (static_cast<int>(starved_task_pool.size()) > starved_topk) {
            std::nth_element(starved_task_pool.begin(),
                             starved_task_pool.begin() + starved_topk,
                             starved_task_pool.end(),
                             by_starved_task);
            starved_task_pool.resize(starved_topk);
        }
        std::sort(starved_task_pool.begin(), starved_task_pool.end(), by_starved_task);
    }
    stats.starved_pool = static_cast<int>(starved_task_pool.size());

    std::vector<std::vector<std::pair<float, int>>> candidates(free_agents.size());

    auto build_start = std::chrono::steady_clock::now();
    int solve_reserve_ms = std::max(40, time_limit_ms / 6);
    if (solve_reserve_ms >= time_limit_ms) solve_reserve_ms = 0;
    auto build_deadline = deadline - std::chrono::milliseconds(solve_reserve_ms);

    auto build_one_agent = [&](int ai) {
        const AgentMeta& agent = free_agents[ai];
        const int pool_keep = std::min(128, std::max(32, topk / 4));
        const int local_keep = std::max(1, topk - pool_keep);
        int chosen_radius = -1;
        int task_count_in_radius = 0;
        for (int radius = local_radius; radius <= max_radius; radius++) {
            task_count_in_radius = 0;
            for (int z = 0; z < zone_count_; z++) {
                if (zone_distance(agent.zone, z) <= radius) {
                    task_count_in_radius += static_cast<int>(tasks_by_zone[z].size());
                }
            }
            if (task_count_in_radius >= target_pool || radius == max_radius) {
                chosen_radius = radius;
                break;
            }
        }

        auto& cand = candidates[ai];
        if (chosen_radius >= 0 && task_count_in_radius > 0) {
            cand.reserve(std::min(task_count_in_radius, topk));
            for (int z = 0; z < zone_count_; z++) {
                if (zone_distance(agent.zone, z) > chosen_radius) continue;
                for (int ti : tasks_by_zone[z]) {
                    if (tasks[ti].component != agent.component) continue;
                    float score = candidate_score(agent, tasks[ti]);
                    cand.push_back({score, ti});
                }
            }
        } else {
            cand.reserve(std::min<int>(tasks.size(), topk));
            for (int ti = 0; ti < static_cast<int>(tasks.size()); ti++) {
                if (tasks[ti].component != agent.component) continue;
                float score = candidate_score(agent, tasks[ti]);
                cand.push_back({score, ti});
            }
        }

        if (static_cast<int>(cand.size()) > local_keep) {
            std::nth_element(cand.begin(), cand.begin() + local_keep, cand.end());
            cand.resize(local_keep);
        }

        std::vector<std::pair<float, int>> pool_cand;
        pool_cand.reserve(std::min<int>(short_task_pool.size() + starved_task_pool.size(),
                                        pool_keep * 2));
        auto append_pool = [&](const std::vector<int>& pool) {
            for (int ti : pool) {
                if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;
                if (tasks[ti].component != agent.component) continue;
                float score = candidate_score(agent, tasks[ti]);
                pool_cand.push_back({score, ti});
            }
        };
        append_pool(short_task_pool);
        append_pool(starved_task_pool);
        if (static_cast<int>(pool_cand.size()) > pool_keep) {
            std::nth_element(pool_cand.begin(), pool_cand.begin() + pool_keep, pool_cand.end());
            pool_cand.resize(pool_keep);
        }
        cand.insert(cand.end(), pool_cand.begin(), pool_cand.end());

        if (static_cast<int>(cand.size()) > topk) {
            std::nth_element(cand.begin(), cand.begin() + topk, cand.end());
            cand.resize(topk);
        }
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end(),
                               [](const auto& lhs, const auto& rhs) {
                                   return lhs.second == rhs.second;
                               }),
                   cand.end());
    };

    int n_build_threads = cfg().num_threads;
    if (n_build_threads <= 0) {
        n_build_threads = static_cast<int>(std::thread::hardware_concurrency());
        if (n_build_threads <= 0) n_build_threads = 4;
    }
    n_build_threads = std::min<int>(n_build_threads, free_agents.size());

    if (n_build_threads > 1 && free_agents.size() >= 256) {
        int chunk = (static_cast<int>(free_agents.size()) + n_build_threads - 1) / n_build_threads;
        std::vector<std::thread> workers;
        workers.reserve(n_build_threads);
        for (int t = 0; t < n_build_threads; t++) {
            int begin = t * chunk;
            int end = std::min<int>(begin + chunk, free_agents.size());
            if (begin >= end) break;
            workers.emplace_back([&, begin, end]() {
                for (int ai = begin; ai < end; ai++) {
                    if (((ai - begin) & 31) == 0 &&
                        std::chrono::steady_clock::now() >= build_deadline) {
                        break;
                    }
                    build_one_agent(ai);
                }
            });
        }
        for (auto& th : workers) th.join();
    } else {
        for (int ai = 0; ai < static_cast<int>(free_agents.size()); ai++) {
            if ((ai & 31) == 0 && std::chrono::steady_clock::now() >= build_deadline) break;
            build_one_agent(ai);
        }
    }

    for (const auto& cand : candidates) {
        if (!cand.empty()) {
            stats.candidate_agents++;
            stats.candidate_total += static_cast<int>(cand.size());
        } else {
            stats.candidate_empty_agents++;
        }
    }
    auto build_end = std::chrono::steady_clock::now();
    stats.build_ms = elapsed_ms(build_start, build_end);

    using HeapItem = std::tuple<float, int, int>; // score, agent-index, candidate-index
    std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<HeapItem>> heap;
    for (int ai = 0; ai < static_cast<int>(candidates.size()); ai++) {
        if (!candidates[ai].empty()) {
            heap.push({candidates[ai][0].first, ai, 0});
        }
    }

    std::vector<unsigned char> task_used(tasks.size(), 0);
    std::vector<unsigned char> agent_used(free_agents.size(), 0);
    std::vector<int> local_schedule = schedule_snapshot_;
    std::vector<int> zone_assign_cap(zone_count_, max_new_assign);
    std::vector<int> zone_assign_used(zone_count_, 0);
    if (cfg().iron_assign_variant == 2) {
        for (int z = 0; z < zone_count_; z++) {
            int local_anchor = zone_free_agents[z] + std::max(6, zone_free_agents[z] / 5);
            int demand_floor = std::max(4, zone_free_tasks[z] / 4);
            zone_assign_cap[z] = std::min(zone_free_tasks[z],
                                          std::max(local_anchor, demand_floor));
        }
    } else if (cfg().iron_assign_variant == 7) {
        int task_total = std::max(1, stats.free_tasks);
        for (int z = 0; z < zone_count_; z++) {
            int fair_share = static_cast<int>(
                (static_cast<long long>(max_new_assign) * zone_free_tasks[z]) / task_total);
            int local_bonus = std::min(zone_free_agents[z] / 4,
                                       std::max(2, zone_free_tasks[z] / 8));
            zone_assign_cap[z] = std::min(zone_free_tasks[z],
                                          std::max(2, fair_share + local_bonus));
        }
    }

    auto can_commit_task = [&](int ti) {
        if (ti < 0 || ti >= static_cast<int>(tasks.size())) return false;
        if (cfg().iron_assign_variant != 2 && cfg().iron_assign_variant != 7) return true;
        int z = tasks[ti].start_zone;
        if (z < 0 || z >= static_cast<int>(zone_assign_cap.size())) return true;
        return zone_assign_used[z] < zone_assign_cap[z];
    };

    int start_dist_gate = H_CAP;
    if (cfg().iron_assign_variant == 5) {
        int t = std::max(0, cur_tick);
        if (t < 2500) {
            start_dist_gate = 50 + t / 50;      // 50 -> 100
        } else if (t < 6000) {
            start_dist_gate = 100 + (t - 2500) / 35; // 100 -> 200
        } else {
            start_dist_gate = H_CAP;
        }
    }

    auto start_gate_allows = [&](int ai, int ti) {
        if (start_dist_gate >= H_CAP) return true;
        if (ai < 0 || ai >= static_cast<int>(free_agents.size())) return false;
        if (ti < 0 || ti >= static_cast<int>(tasks.size())) return false;
        int dist = distance_fast(env, free_agents[ai].loc, tasks[ti].start);
        return dist <= start_dist_gate;
    };

    auto is_opportunity_assignment = [&](int ai, int ti) {
        if (ai < 0 || ai >= static_cast<int>(free_agents.size())) return false;
        if (ti < 0 || ti >= static_cast<int>(tasks.size())) return false;

        const AgentMeta& agent = free_agents[ai];
        const TaskMeta& task = tasks[ti];
        if (task.component != agent.component) return false;

        int dist = distance_fast(env, agent.loc, task.start);
        if (dist > std::max(0, cfg().iron_opportunity_dist)) return false;
        if (dist + task.length > std::max(0, cfg().iron_opportunity_completion)) return false;
        if (zone_distance(agent.zone, task.start_zone) >
            std::max(0, cfg().iron_opportunity_cross)) {
            return false;
        }
        return true;
    };

    auto assignment_window_allows = [&](int ai, int ti) {
        if (cfg().iron_assign_variant != 16) return true;
        if (stats.assigned < base_new_assign) return true;
        if (is_opportunity_assignment(ai, ti)) return true;
        stats.opportunity_rejected++;
        return false;
    };

    auto solve_start = std::chrono::steady_clock::now();

    auto commit_assignment = [&](int ai, int ti) {
        const bool opportunity_extra =
            cfg().iron_assign_variant == 16 && stats.assigned >= base_new_assign;
        const AgentMeta& agent = free_agents[ai];
        const TaskMeta& task = tasks[ti];
        local_schedule[agent.aid] = task.tid;
        task_used[ti] = 1;
        agent_used[ai] = 1;
        if ((cfg().iron_assign_variant == 2 || cfg().iron_assign_variant == 7) &&
            task.start_zone >= 0 &&
            task.start_zone < static_cast<int>(zone_assign_used.size())) {
            zone_assign_used[task.start_zone]++;
        }
        finish_zone_pressure[task.finish_zone]++;
        stats.assigned++;
        if (opportunity_extra) stats.opportunity_assigned++;

        int dist = distance_fast(env, agent.loc, task.start);
        int zd = zone_distance(agent.zone, task.start_zone);
        stats.dist_sum += dist;
        stats.len_sum += task.length;
        stats.completion_cost_sum += dist + task.length;
        stats.zone_dist_sum += zd;
        if (task.length <= 500) stats.assign_len_short++;
        else if (task.length <= 1000) stats.assign_len_mid++;
        else if (task.length <= 1500) stats.assign_len_long++;
        else stats.assign_len_huge++;
        if (dist <= 50) stats.assign_dist_near++;
        else if (dist <= 150) stats.assign_dist_mid++;
        else stats.assign_dist_far++;
        int agent_shortage = (agent.zone >= 0 && agent.zone < static_cast<int>(zone_shortage.size()))
            ? zone_shortage[agent.zone]
            : 0;
        int start_shortage = (task.start_zone >= 0 && task.start_zone < static_cast<int>(zone_shortage.size()))
            ? zone_shortage[task.start_zone]
            : 0;
        int finish_shortage = (task.finish_zone >= 0 && task.finish_zone < static_cast<int>(zone_shortage.size()))
            ? zone_shortage[task.finish_zone]
            : 0;
        stats.start_shortage_sum += start_shortage;
        stats.finish_shortage_sum += finish_shortage;
        stats.migration_gain_sum += finish_shortage - agent_shortage;
        if (zd > 0) stats.cross_zone++;
        if (task.length >= cfg().iron_long_task_th) stats.long_tasks++;
    };

    if (aggressive_assign) {
        const int rounds = std::max(1, cfg().iron_aggressive_iters);
        const int probe_limit = std::max(1, cfg().iron_aggressive_probe);
        const float replace_margin = std::max(0.0f, cfg().iron_aggressive_replace_margin);

        std::vector<int> agent_task(free_agents.size(), -1);
        std::vector<int> task_agent(tasks.size(), -1);
        std::vector<float> agent_cost(free_agents.size(),
                                      std::numeric_limits<float>::infinity());
        std::vector<float> task_cost(tasks.size(),
                                     std::numeric_limits<float>::infinity());
        int assigned_iter = 0;

        auto set_iter_assignment = [&](int ai, int ti) {
            float score = optimization_score(free_agents[ai], tasks[ti]);
            agent_task[ai] = ti;
            agent_cost[ai] = score;
            task_agent[ti] = ai;
            task_cost[ti] = score;
        };

        while (!heap.empty() && assigned_iter < max_new_assign) {
            if ((stats.heap_pops & 255) == 0 &&
                std::chrono::steady_clock::now() >= deadline) {
                break;
            }
            auto [score, ai, ci] = heap.top();
            (void)score;
            heap.pop();
            stats.heap_pops++;

            if (ai < 0 || ai >= static_cast<int>(free_agents.size())) continue;
            if (agent_task[ai] >= 0) continue;
            if (ci < 0 || ci >= static_cast<int>(candidates[ai].size())) continue;

            int ti = candidates[ai][ci].second;
            if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;

            if (!start_gate_allows(ai, ti)) {
                stats.gate_skips++;
                int next_ci = ci + 1;
                if (next_ci < static_cast<int>(candidates[ai].size())) {
                    heap.push({candidates[ai][next_ci].first, ai, next_ci});
                }
                continue;
            }

            if (task_agent[ti] >= 0) {
                stats.lazy_skips++;
                int next_ci = ci + 1;
                if (next_ci < static_cast<int>(candidates[ai].size())) {
                    heap.push({candidates[ai][next_ci].first, ai, next_ci});
                }
                continue;
            }

            set_iter_assignment(ai, ti);
            assigned_iter++;
        }
        stats.aggressive_seeded = assigned_iter;

        for (int ai = 0; ai < static_cast<int>(candidates.size()); ai++) {
            auto& cand = candidates[ai];
            if (cand.empty()) continue;
            for (auto& item : cand) {
                        int ti = item.second;
                        if (ti >= 0 && ti < static_cast<int>(tasks.size())) {
                            item.first = optimization_score(free_agents[ai], tasks[ti]);
                }
            }
            std::sort(cand.begin(), cand.end());
        }

        for (int round = 0; round < rounds; round++) {
            if (std::chrono::steady_clock::now() >= deadline) break;
            int changes = 0;
            stats.aggressive_rounds = round + 1;

            for (int ai = 0; ai < static_cast<int>(free_agents.size()); ai++) {
                if ((ai & 63) == 0 && std::chrono::steady_clock::now() >= deadline) {
                    break;
                }
                const auto& cand = candidates[ai];
                if (cand.empty()) continue;

                if (agent_task[ai] < 0) {
                    int probes = 0;
                    for (const auto& [score, ti] : cand) {
                        if (probes++ >= probe_limit) break;
                        if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;
                        if (tasks[ti].component != free_agents[ai].component) continue;
                        if (!start_gate_allows(ai, ti)) {
                            stats.gate_skips++;
                            continue;
                        }

                        int owner = task_agent[ti];
                        if (owner < 0) {
                            if (assigned_iter >= max_new_assign) continue;
                            set_iter_assignment(ai, ti);
                            assigned_iter++;
                            changes++;
                            break;
                        }

                        if (score + replace_margin < task_cost[ti]) {
                            agent_task[owner] = -1;
                            agent_cost[owner] = std::numeric_limits<float>::infinity();
                            set_iter_assignment(ai, ti);
                            stats.aggressive_replacements++;
                            changes++;
                            break;
                        }
                    }
                    continue;
                }

                int current_ti = agent_task[ai];
                if (current_ti < 0 || current_ti >= static_cast<int>(tasks.size())) continue;
                float current_cost = agent_cost[ai];
                int probes = 0;
                for (const auto& [score, ti] : cand) {
                    if (probes++ >= probe_limit) break;
                    if (ti == current_ti) break;
                    if (score + replace_margin >= current_cost) break;
                    if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;
                    if (tasks[ti].component != free_agents[ai].component) continue;
                    if (!start_gate_allows(ai, ti)) {
                        stats.gate_skips++;
                        continue;
                    }

                    int owner = task_agent[ti];
                    if (owner < 0) {
                        task_agent[current_ti] = -1;
                        task_cost[current_ti] = std::numeric_limits<float>::infinity();
                        set_iter_assignment(ai, ti);
                        stats.aggressive_swaps++;
                        changes++;
                        break;
                    }
                    if (owner == ai) continue;

                    int owner_ti = agent_task[owner];
                    if (owner_ti < 0 || owner_ti >= static_cast<int>(tasks.size())) continue;
                    if (tasks[current_ti].component != free_agents[owner].component) continue;

                    float owner_new = optimization_score(free_agents[owner],
                                                         tasks[current_ti]);
                    if (owner_new >= H_CAP) continue;
                    float old_total = current_cost + agent_cost[owner];
                    float new_total = score + owner_new;
                    if (new_total + replace_margin < old_total) {
                        task_agent[current_ti] = owner;
                        task_cost[current_ti] = owner_new;
                        agent_task[owner] = current_ti;
                        agent_cost[owner] = owner_new;

                        task_agent[ti] = ai;
                        task_cost[ti] = score;
                        agent_task[ai] = ti;
                        agent_cost[ai] = score;

                        stats.aggressive_swaps++;
                        changes++;
                        break;
                    }
                }
            }

            if (changes == 0) break;
        }

        for (int ai = 0;
             ai < static_cast<int>(agent_task.size()) && stats.assigned < max_new_assign;
             ai++) {
            int ti = agent_task[ai];
            if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;
            if (task_used[ti] || agent_used[ai]) continue;
            if (!start_gate_allows(ai, ti)) {
                stats.gate_skips++;
                continue;
            }
            if (!assignment_window_allows(ai, ti)) {
                continue;
            }
            if (!can_commit_task(ti)) {
                stats.quota_skips++;
                continue;
            }
            commit_assignment(ai, ti);
        }
    }

    while (!aggressive_assign && !heap.empty() && stats.assigned < max_new_assign) {
        if ((stats.heap_pops & 255) == 0 && std::chrono::steady_clock::now() >= deadline) break;
        auto [score, ai, ci] = heap.top();
        (void)score;
        heap.pop();
        stats.heap_pops++;

        if (ai < 0 || ai >= static_cast<int>(free_agents.size())) continue;
        if (agent_used[ai]) continue;
        if (ci < 0 || ci >= static_cast<int>(candidates[ai].size())) continue;

        int ti = candidates[ai][ci].second;
        if (ti < 0 || ti >= static_cast<int>(tasks.size())) continue;

        if (!start_gate_allows(ai, ti)) {
            stats.gate_skips++;
            int next_ci = ci + 1;
            if (next_ci < static_cast<int>(candidates[ai].size())) {
                heap.push({candidates[ai][next_ci].first, ai, next_ci});
            }
            continue;
        }

        if (task_used[ti]) {
            stats.lazy_skips++;
            int next_ci = ci + 1;
            if (next_ci < static_cast<int>(candidates[ai].size())) {
                heap.push({candidates[ai][next_ci].first, ai, next_ci});
            }
            continue;
        }
        if (!can_commit_task(ti)) {
            stats.quota_skips++;
            int next_ci = ci + 1;
            if (next_ci < static_cast<int>(candidates[ai].size())) {
                heap.push({candidates[ai][next_ci].first, ai, next_ci});
            }
            continue;
        }
        if (!assignment_window_allows(ai, ti)) {
            int next_ci = ci + 1;
            if (next_ci < static_cast<int>(candidates[ai].size())) {
                heap.push({candidates[ai][next_ci].first, ai, next_ci});
            }
            continue;
        }

        const AgentMeta& agent = free_agents[ai];
        const TaskMeta& task = tasks[ti];
        (void)agent;
        (void)task;
        commit_assignment(ai, ti);
    }

    const int fallback_limit = std::max(256, max_new_assign / 10);
    for (int ai = 0;
         ai < static_cast<int>(free_agents.size()) && stats.assigned < max_new_assign;
         ai++) {
        if (stats.fallback_assigned >= fallback_limit) break;
        if ((ai & 15) == 0 && std::chrono::steady_clock::now() >= deadline) break;
        if (agent_used[ai]) continue;

        const AgentMeta& agent = free_agents[ai];
        int best_ti = -1;
        float best_score = std::numeric_limits<float>::infinity();
        for (int radius = local_radius; radius <= max_radius && best_ti < 0; radius++) {
            for (int z = 0; z < zone_count_; z++) {
                if (zone_distance(agent.zone, z) > radius) continue;
                for (int ti : tasks_by_zone[z]) {
                    if (ti < 0 || ti >= static_cast<int>(tasks.size()) || task_used[ti]) continue;
                    if (tasks[ti].component != agent.component) continue;
                    if (!start_gate_allows(ai, ti)) {
                        stats.gate_skips++;
                        continue;
                    }
                    if (!assignment_window_allows(ai, ti)) continue;
                    if (!can_commit_task(ti)) continue;
                    float score = candidate_score(agent, tasks[ti]);
                    if (score < best_score) {
                        best_score = score;
                        best_ti = ti;
                    }
                }
            }
        }
        if (best_ti >= 0) {
            commit_assignment(ai, best_ti);
            stats.fallback_assigned++;
        } else if (cfg().iron_assign_variant == 2) {
            stats.quota_skips++;
        }
    }

    const int global_pool_limit = std::max(256, max_new_assign / 10);
    std::vector<int> global_pool = short_task_pool;
    global_pool.insert(global_pool.end(), starved_task_pool.begin(), starved_task_pool.end());
    std::sort(global_pool.begin(), global_pool.end());
    global_pool.erase(std::unique(global_pool.begin(), global_pool.end()), global_pool.end());

    for (int ai = 0;
         ai < static_cast<int>(free_agents.size()) && stats.assigned < max_new_assign;
         ai++) {
        if (stats.global_pool_assigned >= global_pool_limit) break;
        if ((ai & 7) == 0 && std::chrono::steady_clock::now() >= deadline) break;
        if (agent_used[ai]) continue;

        const AgentMeta& agent = free_agents[ai];
        int best_ti = -1;
        float best_score = std::numeric_limits<float>::infinity();
        for (int ti : global_pool) {
            if (ti < 0 || ti >= static_cast<int>(tasks.size()) || task_used[ti]) continue;
            if (tasks[ti].component != agent.component) continue;
            if (!start_gate_allows(ai, ti)) {
                stats.gate_skips++;
                continue;
            }
            if (!assignment_window_allows(ai, ti)) continue;
            if (!can_commit_task(ti)) continue;
            float score = candidate_score(agent, tasks[ti]);
            if (score < best_score) {
                best_score = score;
                best_ti = ti;
            }
        }
        if (best_ti >= 0) {
            commit_assignment(ai, best_ti);
            stats.global_pool_assigned++;
        } else if (cfg().iron_assign_variant == 2) {
            stats.quota_skips++;
        }
    }
    auto solve_end = std::chrono::steady_clock::now();
    stats.solve_ms = elapsed_ms(solve_start, solve_end);
    stats.unmatched_agents = std::max(0, stats.free_agents - stats.assigned);

    for (const AgentMeta& agent : free_agents) {
        if (agent.aid >= 0 && agent.aid < static_cast<int>(proposed_schedule.size())) {
            proposed_schedule[agent.aid] = local_schedule[agent.aid];
        }
    }

    int interval = cfg().iron_stats_interval;
    if (interval > 0 && cur_tick - last_stats_tick_ >= interval) {
        last_stats_tick_ = cur_tick;
        print_round_stats(stats, cur_tick);
    }
}

} // namespace optsvr
