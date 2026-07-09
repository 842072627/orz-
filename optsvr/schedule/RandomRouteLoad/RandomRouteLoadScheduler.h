// optsvr/schedule/RandomRouteLoad/RandomRouteLoadScheduler.h
#pragma once

#include "SharedEnv.h"
#include "RuntimeConfig.h"
#include "GreedyHeapMatcher.h"

#include <utility>
#include <vector>

namespace optsvr {

class RandomRouteLoadScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env);

private:
    struct TaskSegmentLoad {
        float load = 0.0f;
        float length = 0.0f;
    };

    struct TaskRouteLoadInfo {
        float avg_load = 0.0f;
        float length = 0.0f;
        float start_load = 0.0f;
        std::vector<TaskSegmentLoad> segments;
    };

    struct RouteZoneEdge {
        int to = -1;
        float length = 0.0f;
    };

    SharedEnvironment* env_ = nullptr;
    int rows_ = 0;
    int cols_ = 0;
    int map_size_ = 0;

    int route_cell_size_ = 4;
    int route_grid_rows_ = 0;
    int route_grid_cols_ = 0;
    int route_grid_count_ = 0;
    std::vector<int> route_zone_rep_locs_;
    std::vector<int> route_zone_free_cells_;
    std::vector<int> route_zone_total_cells_;
    std::vector<float> route_zone_valid_ratio_;
    std::vector<std::vector<RouteZoneEdge>> route_zone_edges_;
    std::vector<float> future_load_;
    std::vector<std::vector<std::pair<int, float>>> zone_pair_corridor_weights_;
    std::vector<float> zone_pair_load_cache_;
    std::vector<float> dynamic_zone_pair_load_cache_;
    std::vector<float> dynamic_zone_pair_len_cache_;
    std::vector<int> dynamic_zone_pair_next_;
    bool dynamic_zone_pair_cache_valid_ = false;

    GreedyHeapMatcher heap_matcher_;

    TaskPool task_pool_snapshot_;
    std::vector<int> curr_locations_snapshot_;
    std::vector<int> schedule_snapshot_;
    int last_route_load_probe_bucket_ = -1;

    void build_route_grid();
    void build_route_zone_graph();
    void build_zone_pair_corridors();
    void refresh_zone_pair_load_cache();
    void refresh_dynamic_zone_pair_cache(float route_weight);
    bool is_free_loc(int loc) const;
    int zone_of_loc(int loc) const;
    int dist(int source, int target) const;
    float zone_capacity_adjusted_load(int zone) const;

    AgentInfo make_agent_info_from_snapshot(int agent_id) const;
    TaskInfo make_task_info_from_snapshot(int task_id) const;

    void reset_future_load(const std::vector<int>& local_schedule);
    void add_agent_task_route_to_load(int agent_id, int task_id, float amount);
    void add_assigned_route_to_load(const AgentInfo& agent, const TaskInfo& task, float amount);
    void add_task_internal_route_to_load(const TaskInfo& task, float amount,
                                         float initial_cumulative);
    void add_segment_corridor_load(int from, int to, float amount);
    void add_zone_pair_static_corridor_load(int z_from, int z_to, float amount);
    bool add_zone_pair_dynamic_corridor_load(int z_from, int z_to, float amount);
    float route_load_decay_factor(float cumulative_dist) const;
    float segment_future_load_amount(float amount, int segment_dist,
                                     float midpoint_dist) const;

    float task_route_load(const TaskInfo& task) const;
    TaskRouteLoadInfo compute_task_route_load_info(const TaskInfo& task) const;
    std::vector<TaskRouteLoadInfo> compute_task_route_load_infos(
        const std::vector<TaskInfo>& tasks) const;
    float pair_route_load(int agent_loc, const TaskInfo& task,
                          const TaskRouteLoadInfo& task_load) const;
    float segment_corridor_load(int from, int to) const;
    float zone_pair_corridor_load(int from_zone, int to_zone) const;
    float zone_pair_route_load(int from_zone, int to_zone) const;
    float point_zone_load(int loc) const;
};

} // namespace optsvr
