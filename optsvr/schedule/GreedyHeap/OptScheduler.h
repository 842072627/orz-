// optsvr/OptScheduler.h — 优化任务调度器
#pragma once

#include "SharedEnv.h"
#include "RuntimeConfig.h"
#include "TaskMatcher.h"
#include "GreedyHeapMatcher.h"
#include <vector>
#include <chrono>
#include <unordered_map>

namespace optsvr {

class OptScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms = 30000);
    void schedule(int time_limit, std::vector<int>& proposed_schedule, SharedEnvironment* env);

    TaskMatcher& matcher() { return matcher_; }
    const TaskMatcher& matcher() const { return matcher_; }

    GreedyHeapMatcher& heap_matcher() { return heap_matcher_; }
    const GreedyHeapMatcher& heap_matcher() const { return heap_matcher_; }

    // 区域拥塞查询（public，供 GreedyHeapMatcher 调用）
    float get_region_congestion(int loc) const;

    // 地图信息（供 GreedyHeapMatcher 预计算密度缓存）
    int cols() const { return cols_; }

private:
    // 地图基本信息
    int map_size_ = 0;
    int rows_ = 0;
    int cols_ = 0;

    // ---- Grid-based 拥塞系统 ----
    int grid_rows_ = 0;
    int grid_cols_ = 0;
    int grid_count_ = 0;
    std::vector<float> grid_congestion_;

    int cell_to_grid(int loc) const;

    // ---- 核心步骤 ----
    void build_grid_layout();

    // ---- 匹配器 ----
    TaskMatcher matcher_;
    GreedyHeapMatcher heap_matcher_;

    // ---- 快照（防止多线程竞争） ----
    // schedule() 入口处一次性拷贝，后续所有操作只读快照
    TaskPool task_pool_snapshot_;
    std::vector<int> curr_locations_snapshot_;   // agent locations 快照
    std::vector<int> schedule_snapshot_;         // proposed_schedule 快照

    // ---- 辅助 ----
    AgentInfo make_agent_info_from_snapshot(int agent_id) const;
    TaskInfo  make_task_info_from_snapshot(int task_id) const;
    void update_congestion_from_snapshot(int n_agents);
};

} // namespace optsvr
