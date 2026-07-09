// optsvr/schedule/GameOpt/GameOptScheduler.h — Game 赛道定制调度器
#pragma once

#include "SharedEnv.h"
#include "RuntimeConfig.h"
#include "TaskMatcher.h"
#include "GreedyHeapMatcher.h"
#include <vector>
#include <chrono>
#include <unordered_map>

namespace orz_engine {

class GameOptScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env);

private:
    // 参数（由 load_game_params 从 cfg().game_* 加载）
    float dist_weight_    = 0;
    int   rebuild_pct_    = 0;
    int   lns_pct_        = 0;
    int   sort_k_         = 0;
    bool  reassign_on_    = false;
    float keep_bias_      = 0;
    int   min_dist_       = 0;
    float cong_decay_     = 0;
    int   grid_cell_      = 0;
    int   grid_radius_    = 0;
    float dest_cong_weight_ = 0;
    float dest_cong_cap_    = 0;

    int map_size_ = 0;
    int rows_ = 0;
    int cols_ = 0;

    int grid_rows_ = 0;
    int grid_cols_ = 0;
    int grid_count_ = 0;
    std::vector<float> grid_congestion_;
    std::vector<float> grid_dest_load_;

    int cell_to_grid(int loc) const;
    void build_grid_layout();
    float get_region_congestion(int loc) const;
    float get_dest_congestion(int loc) const;
    void  update_dest_load(int n_agents);

    GreedyHeapMatcher heap_matcher_;

    TaskPool task_pool_snapshot_;
    std::vector<int> curr_locations_snapshot_;
    std::vector<int> schedule_snapshot_;

    AgentInfo make_agent_info(int agent_id) const;
    TaskInfo  make_task_info(int task_id) const;
    void update_congestion(int n_agents);
    void load_game_params();
};

} // namespace orz_engine
