// optsvr/schedule/GameBalanceOpt/GameBalanceOptScheduler.h
// 中心线zone调度器 — BFS中心线 + zone密度 + 方向锁 + 三层豁免打分
#pragma once

#include "SharedEnv.h"
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

namespace optsvr {

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
    std::vector<ZoneMainRange> zone_ranges_;
    std::vector<int> zone24_density_;    // [zid] → 实时 agent 计数
    std::vector<int> zone_is_main_;      // [zid] → 1=中心线经过
    std::vector<int> zone_direction_;    // [zid] → 方向: +1/-1/0
    std::vector<int> agent_last_main_;   // [aid] → 上一 tick 的 main_dist
    std::vector<int> parked_task_;       // [aid] → 停放的多 errand task id, -1=无

    // ---- agent空闲计数器（IDEA4: idle exempt）----
    std::vector<int> agent_idle_ticks_;  // [aid] → 连续空闲tick数

    // ---- IDEA: zone width (bottleneck awareness) ----
    std::vector<int> zone_min_width_;    // [zid] → minimum cross-section width

    int step_ = 0;

    // ---- 中心线初始化 ----
    void build_centerline(const SharedEnvironment* env);
    void compute_dtw(const SharedEnvironment* env, std::vector<int>& dtw);
    void build_zone_ranges();
    void compute_zone_widths(const SharedEnvironment* env, const std::vector<int>& dtw);

    // ---- 每 tick 更新 ----
    void update_zone_density(const SharedEnvironment* env);
    void update_zone_direction(const SharedEnvironment* env);

    // ---- 投影与打分 ----
    void project(int loc, int& m, int& b) const;
    float base_score(int a_loc, int tid, int step, int start_idx = 0) const;
    float base_score_ex(int a_loc, int tid, int step, int start_idx,
                        int eff_density_cap) const;
    float connection_exempt_score(int agent_loc, const std::vector<int>& task_locs) const;
    bool  is_hot_zone_exempt(int loc) const;

    // ---- 辅助 ----
    static inline int zid_100(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }
    static inline int zh_dist(int z1, int z2) {
        return std::abs(z1/100 - z2/100) + std::abs(z1%100 - z2%100);
    }
};

} // namespace optsvr
