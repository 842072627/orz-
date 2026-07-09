// optsvr/schedule/DHSchedule/DHScheduler.h
// 来源：IDEA/Schedule-DH，用类封装，不改逻辑结构
// 原始文件对应关系：
//   DHScheduler.cpp       ← TaskScheduler.cpp (init/plan/schedule_plan + 工具函数)
//   DHSchedulerLargeR.cpp ← TaskSchedulerLargeR.cpp (K-means 聚类)
//   DHSchedulerOW.cpp     ← TaskSchedulerOW.cpp (QuadTree + planOW)
#pragma once

#include "SharedEnv.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <random>
#include <utility>

namespace orz_engine {

class DHScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env);

private:
    SharedEnvironment* env_ = nullptr;

    // ---- 原全局变量，改为类成员 ----
    std::unordered_set<int> free_agents_;
    std::unordered_set<int> free_tasks_;
    std::unordered_map<int, int> task_assignments_;
    std::unordered_map<int, int> location_congestion_;
    std::mt19937 mt_{0};

    // ---- 距离查询（替代 PrecomputedHeuristicTable） ----
    int scheduler_distance(int source, int target) const;
    int task_path_cost(int agent_loc, const Task& task) const;

    // ---- 来自 TaskScheduler.cpp 的工具函数 ----
    static constexpr int MAX_CLUSTER_AGENTS = 5;
    static constexpr int MAX_CLUSTER_TASKS  = 10;
    static constexpr int MIN_RADIUS = 5;
    static constexpr int MAX_RADIUS = 50;
    static constexpr int BASE_RADIUS = 20;
    static constexpr double DENSITY_LOW_THRESHOLD = 0.5;
    static constexpr double DENSITY_HIGH_THRESHOLD = 2.0;
    static constexpr int K_BEST_TASKS = 5;

    int computeDynamicRadius(int num_free_agents, int num_free_tasks);
    int computeMakespanHelper(int agent_id, int task_id);
    int hungarianMinMatch(const std::vector<std::vector<int>>& cost, std::vector<int>& assignment);

    // 匈牙利浮点版（内联类）
    double hungarianSolve(std::vector<std::vector<double>>& cost_matrix, std::vector<int>& assignment);

    // reclaim 相关
    int optimistic_ticks_to_first_loc(int agent_id, const Task& task);
    bool can_reclaim_task(int agent_id, const Task& task);
    void rebuild_scheduler_state(std::vector<int>& proposed_schedule);
    void reclaim_safe_tasks(std::vector<int>& proposed_schedule);

    // schedule_plan（maze/大图策略）
    void schedule_plan(int time_limit, std::vector<int>& proposed_schedule);

    // ---- 来自 TaskSchedulerLargeR.cpp ----
    struct Cluster {
        std::vector<int> task_ids;
        std::pair<int, int> center;
    };
    std::vector<Cluster> clusters_;
    std::unordered_map<int, int> task_to_cluster_;
    int last_clustered_free_tasks_count_ = 0;
    std::mt19937 rng_{42};

    std::pair<int, int> get_task_location(int task_id);
    double euclidean_distance(const std::pair<int, int>& a, const std::pair<int, int>& b);
    void cluster_tasks();
    int find_nearest_cluster(int agent_id);
    void planLargeR(int time_limit, std::vector<int>& proposed_schedule);

    // ---- 来自 TaskSchedulerOW.cpp ----
    struct QuadTreeNode {
        int x_min, y_min, x_max, y_max;
        int level;
        std::vector<int> agent_indices;
        std::vector<int> task_ids;
        QuadTreeNode* children[4];
        bool is_leaf;
        QuadTreeNode(int x1, int y1, int x2, int y2, int lvl);
        ~QuadTreeNode();
        bool contains(int loc, int cols) const;
        int getChildIndex(int loc, int cols) const;
        void subdivide(int cols);
    };

    QuadTreeNode* quad_root_ = nullptr;
    int max_depth_ = 6;
    int max_items_per_leaf_ = 8;
    int map_cols_ = 100;
    int map_rows_ = 100;

    void initQuadTree();
    void rebuildQuadTree();
    void insertAgentInternal(int agent_id, QuadTreeNode* node);
    void insertTaskInternal(int task_id, QuadTreeNode* node);
    void collectTasksInRadius(int agent_loc, int radius, std::vector<int>& nearby_tasks);
    int estimateTaskMakespan(int agent_id, int task_id) const;
    int getLocationX(int loc) const;
    int getLocationY(int loc) const;
    void planOW(int time_limit, std::vector<int>& proposed_schedule);
};

} // namespace orz_engine
