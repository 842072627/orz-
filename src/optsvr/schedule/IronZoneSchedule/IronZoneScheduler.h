// optsvr/schedule/IronZoneSchedule/IronZoneScheduler.h
// IRON large-map scheduler: zone-local greedy matching with future-flow diagnostics.
#pragma once

#include "SharedEnv.h"
#include <chrono>
#include <cstdint>
#include <vector>

namespace optsvr {

class IronZoneScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule,
                  SharedEnvironment* env);
    void idle_relocate(const std::vector<int>& proposed_schedule,
                       SharedEnvironment* env, bool log = false);

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
        bool ff_blocked = false;
    };

    struct RoundStats {
        int free_agents = 0;
        int free_tasks = 0;
        int busy_before = 0;
        int assigned = 0;
        int unmatched_agents = 0;
        int skipped_component = 0;
        int skipped_highway_agent = 0;
        long long dist_sum = 0;
        long long len_sum = 0;
        long long zone_dist_sum = 0;
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

    // ---- future flow data ----
    int ff_zone_rows_ = 1;
    int ff_zone_cols_ = 1;
    int ff_zone_count_ = 1;
    std::vector<std::vector<int>> ff_zone_adj_;      // 预计算ff zone邻接表
    std::vector<float> ff_dir_;                      // 方向flow, size=ff_zone_count_*4
    std::vector<uint16_t> ff_path_next_;             // next_hop[src*dst], size=ff_zone_count_^2
    std::vector<uint16_t> ff_path_dist_;             // dist[src*dst], size=ff_zone_count_^2
    bool ff_path_ready_ = false;

    std::vector<int> ff_bfs_parent_;                 // BFS parent (初始化时用)
    std::vector<uint16_t> ff_bfs_dist_;              // BFS dist (初始化时用)
    int ff_bfs_cached_source_ = -1;                  // BFS缓存的source zone
    
    static constexpr int DIR_E = 0, DIR_S = 1, DIR_W = 2, DIR_N = 3;
    int ff_dir_from_delta(int dr, int dc) const;
    
    int ff_zone_of_loc(int loc) const;
    void ff_compute_zone_adjacency(SharedEnvironment* env);
    void ff_precompute_all_pairs(SharedEnvironment* env);
    void ff_save_cache(const char* path) const;
    bool ff_load_cache(const char* path);
    std::vector<int> ff_get_zone_path_fast(int z_from, int z_to) const;
    std::vector<int> ff_get_zone_path(int z_from, int z_to);
    void ff_add_future_flow_path(int z_from, int z_to, float weight, int cumul_offset);
    int ff_cell_size() const { return std::max(1, rows_ / std::max(1, ff_zone_rows_)); }

    int zone_of_loc(int loc) const;
    int component_of_loc(int loc) const;
    void build_components(SharedEnvironment* env);
    bool task_remaining_single_component(const Task& task, int start_idx,
                                         int* component) const;
    int zone_distance(int za, int zb) const;
    int distance_fast(SharedEnvironment* env, int source, int target) const;
    int effective_tick(SharedEnvironment* env) const;
    int remaining_task_length(SharedEnvironment* env, const Task& task) const;
    int first_remaining_index(const Task& task) const;
    
    // future flow
    void print_round_stats(const RoundStats& stats, int tick) const;
    mutable int ff_locked_candidates_ = 0;
};

} // namespace optsvr
