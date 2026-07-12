// optsvr/schedule/GameBalanceOpt/GameBalanceOptScheduler.h
// 中心线zone调度器 — BFS中心线 + zone密度 + 方向锁 + 三层豁免打分
#pragma once

#include "SharedEnv.h"
#include "PibtLock.h"
#include "RuntimeConfig.h"
#include "PibtLock.h"
#include <vector>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <queue>
#include <cmath>
#include <algorithm>
#include <climits>
#include <cstdio>
#include <random>
#include <cstdint>
#include <deque>
#include <string>
#include <array>

namespace nz {

class GameBalanceOptScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env);

private:
    SharedEnvironment* env_ = nullptr;

    // ---- 地图基本信息 ----
    int rows_ = 0, cols_ = 0, total_ = 0;
    int n24c_ = 0, n24r_ = 0;  // zone 网格列数/行数

    // ---- 中心线数据 ----
    bool ready_ = false;
    int  total_len_ = 0;
    std::vector<int> main_dist_;         // [loc] → 主线距离
    std::vector<int> branch_dist_;       // [loc] → 分支距离
    std::vector<int> centerline_cells_;  // 中心线格子列表

    // ---- Zone 数据 ----
    struct ZoneMainRange {
        int zone_id;
        int min_m, max_m;  // 中心线 main_dist 范围
        int lock_dir;      // 方向锁: +1/-1/0
    };
    enum class ScoreRejectReason : unsigned char {
        None = 0,
        Missing,
        Channel,
        FutureFlow,
        LockDir,
        Density,
        Branch,
        Other,
    };
    std::vector<ZoneMainRange> zone_ranges_;
    std::vector<int> zone24_density_;    // [zid] → 实时 assigned-agent 计数
    std::vector<unsigned char> zone24_has_free_; // [zid] → zone内是否存在可行走格
    std::vector<int> zone24_valid_cells_; // [zid] → zone内实际可行走格数量
    std::vector<std::vector<int>> zone24_neighbors_; // actual cross-boundary connectivity
    std::vector<uint16_t> zone24_dist_;  // [src*nz+dst] → zone级BFS距离
    std::vector<int> zone_is_main_;      // [zid] → 1=中心线经过
    std::vector<int> zone_direction_;    // [zid] → 方向: +1/-1/0
    std::vector<int> agent_last_main_;   // [aid] → 上一 tick 的 main_dist
    std::vector<int> parked_task_;       // [aid] → 停放的多 errand task id, -1=无

    // ---- agent空闲计数器（IDEA4: idle exempt）----
    std::vector<int> agent_idle_ticks_;  // [aid] → 连续空闲tick数
    int last_idle_update_timestep_ = -1;

    std::vector<float> zone_future_pos_; // [zid] → decayed future-flow in +main direction
    std::vector<float> zone_future_neg_; // [zid] → decayed future-flow in -main direction
    std::vector<float> zone_future_exempt_pos_; // horizon外独立预留流量(+main)
    std::vector<float> zone_future_exempt_neg_; // horizon外独立预留流量(-main)
    struct ZoneStallSample {
        int start_tick = 0;
        int end_tick = 0;
        std::vector<uint16_t> stationary_agents;
        std::vector<uint16_t> stationary_busy_agents;
    };
    std::deque<ZoneStallSample> zone_stall_samples_;
    std::vector<double> zone_stall_agent_ticks_;
    std::vector<double> zone_stall_busy_agent_ticks_;
    std::vector<float> zone_stall_density_; // total stationary density, diagnostics only
    std::vector<float> zone_stall_busy_density_; // busy-agent signal used by dynamic cap
    std::vector<float> zone_dynamic_cap_;
    std::vector<int> dynamic_cap_prev_agent_loc_;
    int dynamic_cap_prev_tick_ = -1;
    int dynamic_cap_last_update_tick_ = -1;
    int dynamic_cap_last_window_ticks_ = 0;
    int dynamic_cap_last_up_ = 0;
    int dynamic_cap_last_down_ = 0;
    float dynamic_cap_last_avg_ = 0.0f;
    float dynamic_cap_last_min_ = 0.0f;
    float dynamic_cap_last_max_ = 0.0f;
    float dynamic_cap_last_stall_avg_ = 0.0f;
    float dynamic_cap_last_stall_max_ = 0.0f;
    float dynamic_cap_last_busy_stall_avg_ = 0.0f;
    float dynamic_cap_last_free_stall_avg_ = 0.0f;
    int dynamic_cap_last_active_ = 0;
    int dynamic_cap_last_below_base_ = 0;
    int dynamic_cap_last_above_base_ = 0;
    int dynamic_cap_last_at_min_ = 0;
    mutable std::mt19937 ff_rng_{20260709u};
    std::mt19937 idle_global_rng_{20260710u};
    std::mt19937 short_task_rng_{20260712u};
    bool short_task_length_cuts_ready_ = false;
    std::array<int, 4> short_task_length_cuts_{{0, 0, 0, 0}};
    std::array<float, 5> short_task_length_means_{{1, 1, 1, 1, 1}};

    struct TaskStaticCache {
        float intrinsic_distance = 0.0f; // errand0->errand1->...，不含agent pickup
        float coarse_intrinsic_distance = 0.0f;
        bool distance_valid = true;
        bool coarse_distance_valid = true;
        bool distance_ready = false;
        bool zone_paths_ready = false;
        // tail_distance[i] = locations[i]之后所有内部errand段的距离和。
        std::vector<float> tail_distance;
        // Projected-distance fallback with the same tail semantics.
        std::vector<float> coarse_tail_distance;
        // [i] caches the zone path locations[i-1] -> locations[i]; index 0 unused.
        std::vector<std::vector<int>> internal_zone_paths;
    };
    std::unordered_map<int, TaskStaticCache> task_static_cache_;

    int step_ = 0;

    // ---- 中心线初始化 ----
    void build_centerline(const SharedEnvironment* env);
    void compute_dtw(const SharedEnvironment* env, std::vector<int>& dtw);
    void build_zone_ranges();

    // ---- 每 tick 更新 ----
    void update_zone_density(const SharedEnvironment* env);
    void update_zone_direction(const SharedEnvironment* env);

    // ---- 投影与打分 ----
    void project(int loc, int& m, int& b) const;
    float projected_distance_between(int from, int to) const;
    float score_distance_between(int from, int to) const;
    float base_score(int a_loc, int tid, int step, int start_idx = 0) const;
    float base_score_ex(int a_loc, int tid, int step, int start_idx,
                        int eff_density_cap,
                        ScoreRejectReason* reject_reason = nullptr) const;
    float connection_exempt_score(int agent_loc, const std::vector<int>& task_locs) const;
    bool  is_hot_zone_exempt(int loc) const;
    bool  game_directional_flow_enabled() const;
    bool  task_has_channel_errand(const std::vector<int>& locs) const;
    int   zone24_of_loc(int loc) const;
    void  build_zone24_distances();
    std::vector<int> shortest_zone_path(int from_zid, int to_zid,
                                        bool randomize) const;
    bool build_task_zone_cache(int tid, TaskStaticCache& out) const;
    bool build_task_distance_cache(int tid, TaskStaticCache& out) const;
    bool build_task_static_cache(int tid, TaskStaticCache& out) const;
    std::size_t prewarm_task_static_cache(
        const std::vector<int>& tids,
        std::chrono::steady_clock::time_point deadline,
        int max_threads = 0);
    const TaskStaticCache* ensure_task_static_cache(int tid);
    const std::vector<int>* cached_internal_zone_path(
        int tid, int edge_idx, int from_loc, const std::vector<int>& locs) const;
    bool  directional_future_flow_allows(int agent_loc,
                                         const std::vector<int>& locs,
                                         int start_idx,
                                         int tid = -1) const;
    float directional_future_flow_soft_penalty(int agent_loc,
                                               const std::vector<int>& locs,
                                               int start_idx,
                                               int tid = -1) const;
    void  reset_directional_future_flow();
    void  seed_directional_future_flow(const SharedEnvironment* env);
    void  reset_dynamic_zone_cap(const SharedEnvironment* env);
    void  update_dynamic_zone_cap(const SharedEnvironment* env);
    float directional_zone_cap(int zid, int intent) const;
    void  add_directional_future_flow_for_task(int agent_loc, int tid,
                                               int start_idx = -1);
    void  adjust_directional_future_flow_for_task(int agent_loc, int tid,
                                                  int start_idx, int weight);
    void  add_directional_future_flow_route(int agent_loc,
                                            const std::vector<int>& locs,
                                            int start_idx,
                                            int weight = 1,
                                            int tid = -1);

    // ---- 辅助 ----
    static inline int zid_100(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
    static inline int zh_dist(int z1, int z2) {
        return std::abs(z1/100 - z2/100) + std::abs(z1%100 - z2%100);
    }
};

} // namespace nz
