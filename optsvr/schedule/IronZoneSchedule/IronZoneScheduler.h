// optsvr/schedule/IronZoneSchedule/IronZoneScheduler.h
// IRON large-map scheduler: zone-local candidate generation + lazy heap matching.
#pragma once

#include "SharedEnv.h"
#include <chrono>
#include <vector>

namespace optsvr {

class IronZoneScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule,
                  SharedEnvironment* env);

private:
    struct AgentMeta {
        int aid = -1;
        int loc = -1;
        int zone = -1;
        int component = -1;
    };

    struct TaskMeta {
        int tid = -1;
        int start = -1;
        int finish = -1;
        int start_zone = -1;
        int finish_zone = -1;
        int component = -1;
        int length = 0;
        int age = 0;
    };

    struct RoundStats {
        int free_agents = 0;
        int free_tasks = 0;
        int busy_before = 0;
        int assigned = 0;
        int candidate_agents = 0;
        int candidate_total = 0;
        int heap_pops = 0;
        int lazy_skips = 0;
        int fallback_assigned = 0;
        int global_pool_assigned = 0;
        int aggressive_rounds = 0;
        int aggressive_seeded = 0;
        int aggressive_replacements = 0;
        int aggressive_swaps = 0;
        int assign_variant = 0;
        int target_active = 0;
        int base_target_active = 0;
        int opportunity_cap = 0;
        int opportunity_assigned = 0;
        int opportunity_rejected = 0;
        int quota_skips = 0;
        int gate_skips = 0;
        int candidate_empty_agents = 0;
        int unmatched_agents = 0;
        int skipped_component = 0;
        int starved_zones = 0;
        int short_pool = 0;
        int starved_pool = 0;
        long long dist_sum = 0;
        long long len_sum = 0;
        long long zone_dist_sum = 0;
        long long start_shortage_sum = 0;
        long long finish_shortage_sum = 0;
        long long migration_gain_sum = 0;
        long long completion_cost_sum = 0;
        int cross_zone = 0;
        int long_tasks = 0;
        int free_len_short = 0;
        int free_len_mid = 0;
        int free_len_long = 0;
        int free_len_huge = 0;
        int assign_len_short = 0;
        int assign_len_mid = 0;
        int assign_len_long = 0;
        int assign_len_huge = 0;
        int assign_dist_near = 0;
        int assign_dist_mid = 0;
        int assign_dist_far = 0;
        double early_phase = 0.0;
        double build_ms = 0.0;
        double solve_ms = 0.0;
    };

    SharedEnvironment* env_ = nullptr;
    int rows_ = 0;
    int cols_ = 0;
    int map_size_ = 0;
    int zone_rows_ = 1;
    int zone_cols_ = 1;
    int zone_count_ = 1;
    int last_stats_tick_ = -1000000000;
    std::vector<int> component_id_;
    int component_count_ = 0;

    std::vector<int> curr_locations_snapshot_;
    std::vector<int> schedule_snapshot_;
    TaskPool task_pool_snapshot_;

    int zone_of_loc(int loc) const;
    int component_of_loc(int loc) const;
    void build_components(SharedEnvironment* env);
    bool task_remaining_single_component(const Task& task, int start_idx,
                                         int* component) const;
    int zone_distance(int za, int zb) const;
    int distance_fast(SharedEnvironment* env, int source, int target) const;
    int effective_tick(SharedEnvironment* env) const;
    float early_phase_factor() const;
    int remaining_task_length(SharedEnvironment* env, const Task& task) const;
    int first_remaining_index(const Task& task) const;
    float score_candidate(const AgentMeta& agent, const TaskMeta& task,
                          const std::vector<int>& finish_zone_pressure,
                          const std::vector<int>& zone_shortage) const;
    void print_round_stats(const RoundStats& stats, int tick) const;
};

} // namespace optsvr
