// optsvr/schedule/GameBalanceOpt/PibtLock.h
// Game bottleneck blackboard shared by scheduler, planner, and executor.
#pragma once

#include <cstring>

namespace optsvr {

struct PibtLock {
    static constexpr int MAX_SEGMENTS = 1024;
    static constexpr int MAX_AGENTS   = 10000;
    static constexpr int MAX_ZONES    = 4096;  // 24x24 zone grid

    int seg_lock_dir[MAX_SEGMENTS];   // zone 方向锁: +1/-1/0
    int agent_blocked[MAX_AGENTS];    // 1=PIBT 应原地等, 0=正常
    int parked_task[MAX_AGENTS];      // 停放的第二 errand task id, -1=无

    // ---- IDEA: gradual priority penalty ----
    float agent_priority_penalty[MAX_AGENTS]; // 0.0=正常, 负值=降低优先级

    // ---- IDEA: zone direction for PIBT counter-flow ----
    int zone_dir[MAX_ZONES];          // zone方向锁: +1/-1/0 (供PIBT查询)
    int zone_n24c;                    // zone grid列数 (供PIBT计算zone id)

    // ---- IDEA10: zone width for flow bottleneck ----
    int zone_width[MAX_ZONES];        // zone最小宽度 (供Planner查询)

    // ---- Game narrow passage: cell-level projection shared with Planner ----
    const int* main_dist_ptr = nullptr;   // [loc] -> main_dist, owned by scheduler
    const int* branch_dist_ptr = nullptr; // [loc] -> branch_dist, owned by scheduler
    const int* zone_density_ptr = nullptr; // [zone] -> current agent count, owned by scheduler
    int total_cells = 0;

    // ---- Blackboard diagnostics ----
    int static_profile_epoch = 0;
    int runtime_epoch = 0;
    long long planner_blocked_waits = 0;
    long long planner_narrow_gate_checks = 0;
    long long planner_counter_flow_penalty_hits = 0;
    long long planner_counter_flow_penalty_cost = 0;
    int planner_r6_active_idea = 0;
    long long planner_r6_hot_zone_hits = 0;
    long long planner_r6_successor_adjustments = 0;
    long long planner_r6_exit_bonus_hits = 0;
    long long planner_r6_entry_penalty_hits = 0;
    long long planner_r6_wait_bias_hits = 0;

    void reset_runtime_signals() {
        std::memset(seg_lock_dir, 0, sizeof(seg_lock_dir));
        std::memset(agent_blocked, 0, sizeof(agent_blocked));
        std::memset(parked_task, 0xFF, sizeof(parked_task)); // -1
        std::memset(agent_priority_penalty, 0, sizeof(agent_priority_penalty));
        std::memset(zone_dir, 0, sizeof(zone_dir));
        runtime_epoch++;
    }

    void reset_static_profile() {
        std::memset(zone_width, 0, sizeof(zone_width));
        zone_n24c = 0;
        main_dist_ptr = nullptr;
        branch_dist_ptr = nullptr;
        zone_density_ptr = nullptr;
        total_cells = 0;
        static_profile_epoch++;
    }

    void reset_planner_diagnostics() {
        planner_blocked_waits = 0;
        planner_narrow_gate_checks = 0;
        planner_counter_flow_penalty_hits = 0;
        planner_counter_flow_penalty_cost = 0;
        planner_r6_active_idea = 0;
        planner_r6_hot_zone_hits = 0;
        planner_r6_successor_adjustments = 0;
        planner_r6_exit_bonus_hits = 0;
        planner_r6_entry_penalty_hits = 0;
        planner_r6_wait_bias_hits = 0;
    }

    void reset_all() {
        reset_runtime_signals();
        reset_static_profile();
        reset_planner_diagnostics();
    }

    // Backward-compatible full reset. New code should prefer the explicit
    // reset_* methods so scheduler-owned static profile is not wiped by planner.
    void reset() {
        reset_all();
    }

    void clear_agent_runtime(int n_agents) {
        if (n_agents < 0 || n_agents > MAX_AGENTS) n_agents = MAX_AGENTS;
        for (int a = 0; a < n_agents; a++) {
            agent_blocked[a] = 0;
            parked_task[a] = -1;
            agent_priority_penalty[a] = 0.0f;
        }
        runtime_epoch++;
    }

    void clear_zone_runtime(int n_zones, int n_segments) {
        if (n_zones < 0 || n_zones > MAX_ZONES) n_zones = MAX_ZONES;
        if (n_segments < 0 || n_segments > MAX_SEGMENTS) n_segments = MAX_SEGMENTS;
        for (int zi = 0; zi < n_zones; zi++) zone_dir[zi] = 0;
        for (int si = 0; si < n_segments; si++) seg_lock_dir[si] = 0;
        runtime_epoch++;
    }

    void install_static_profile(int n24c,
                                const int* main_dist,
                                const int* branch_dist,
                                const int* zone_density,
                                int cells) {
        zone_n24c = n24c;
        main_dist_ptr = main_dist;
        branch_dist_ptr = branch_dist;
        zone_density_ptr = zone_density;
        total_cells = cells;
        static_profile_epoch++;
    }

    bool static_profile_ready() const {
        return zone_n24c > 0 &&
               total_cells > 0 &&
               main_dist_ptr != nullptr &&
               branch_dist_ptr != nullptr;
    }

    int zone_width_nonzero_count() const {
        int count = 0;
        for (int zi = 0; zi < MAX_ZONES; zi++) {
            if (zone_width[zi] > 0) count++;
        }
        return count;
    }

    void record_planner_blocked_wait() {
        planner_blocked_waits++;
    }

    void record_planner_narrow_gate_check() {
        planner_narrow_gate_checks++;
    }

    void record_planner_counter_flow_penalty(int cost) {
        planner_counter_flow_penalty_hits++;
        planner_counter_flow_penalty_cost += cost;
    }

    void set_planner_r6_active_idea(int idea) {
        planner_r6_active_idea = idea;
    }

    void record_planner_r6_hot_zone_hit() {
        planner_r6_hot_zone_hits++;
    }

    void record_planner_r6_successor_adjustment() {
        planner_r6_successor_adjustments++;
    }

    void record_planner_r6_exit_bonus() {
        planner_r6_exit_bonus_hits++;
        record_planner_r6_successor_adjustment();
    }

    void record_planner_r6_entry_penalty() {
        planner_r6_entry_penalty_hits++;
        record_planner_r6_successor_adjustment();
    }

    void record_planner_r6_wait_bias() {
        planner_r6_wait_bias_hits++;
        record_planner_r6_successor_adjustment();
    }

    static PibtLock& instance() {
        static PibtLock inst;
        return inst;
    }
private:
    PibtLock() { reset(); }
};

inline PibtLock& pibt_lock() { return PibtLock::instance(); }

} // namespace optsvr
