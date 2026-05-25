// config.h - Combined Track global configuration
// All map strategy selection controlled here

#pragma once
#include <string>
#include "SharedEnv.h"

// ========== Map detection ==========
inline bool is_ORZ(SharedEnvironment* env) {
    return env->map_name.find("orz") != std::string::npos;
}
inline bool is_BOS(SharedEnvironment* env) {
    return env->map_name.find("bos") != std::string::npos ||
           env->map_name.find("Boston") != std::string::npos;
}
inline bool is_FULFILL(SharedEnvironment* env) {
    return env->map_name.find("fulfill") != std::string::npos;
}
inline bool has_custom_strategy(SharedEnvironment* env) {
    return is_ORZ(env);  // only ORZ has custom strategy
}

// ========== Strategy selection ==========
#define ORZ_SCHEDULER   "centerline"    // ORZ scheduler: centerline heatmap + greedy match (v2)
#define ORZ_PLANNER     "default"       // ORZ planner: default PIBT
#define ORZ_EXECUTOR    "default"       // ORZ executor: default

// ========== Future flow (REMOVED, replaced by USE_TRAFFIC_PREDICTION) ==========

// ========== Traffic prediction ==========
#define USE_TRAFFIC_PREDICTION  1
#define PREDICT_SPEED          1.5f
#define NARROW_WIDTH_MAX       5

// ========== Direction lock thresholds ==========
#define HEAT_UNLOCK_THRESHOLD    7    // 组内MP热度低于此值 → 解锁
#define HEAT_AGENT_TASK_REJECT  20    // 逆向取任务时经过MP的最大热度
#define HEAT_STOP_FAIL          30    // 热度高于此值 → 截停失效
#define GROUP_THRESHOLD        200    // 窄段分组最大 main_dist 距离
#define WIDE_BREAK              15    // 宽度>此值 → 分组断开
#define USE_DUMMY_GOAL        false   // 空闲agent回spawn

// ========== Task segment distance limit ==========
#define MAX_TASK_SEGMENT_DIST  99999   // max main+branch distance for a single errand transition
#define GARBAGE_TASK_RATIO      0.0f  // bottom fraction of free tasks deferred each tick (0.0-1.0)

// ========== Hard density limit ==========
#define USE_DENSITY_HARD_CAP  true    // enable hard density cap
#define DENSITY_HARD_CAP      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  false    // scheduler + PIBT direction block linkage

// ========== Density exclusion ==========
#define USE_DENSITY_EXCLUDE_IDLE  true    // 排除无任务/被截停agent的密度贡献

// ========== Wall-hugging in corridors (PIBT tie-breaker + A* guide path) ==========
#define USE_WALL_HUGGING        true    // master switch for all wall-hugging features
#define USE_WALL_HUGGING_PIBT    true    // PIBT successor tie-breaker wall-hugging
#define USE_WALL_HUGGING_ASTAR   true    // A* guide path branch-dist bias
#define WALL_HUG_WIDTH_MAX       20      // max corridor width (cells) for wall-hugging
#define WALL_HUG_BRANCH_BIAS_MAX 2       // max per-step branch bias for A* (0-5, larger = stronger)

// ========== Idle capacity control ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

// ========== Migration flags ==========
#define USE_MP_CENTERLINE   1   // widest path centerline
#define USE_MP_DENSITY      1   // 1D main point density (heatmap)
#define USE_MP_WIDTH        1   // P10 corridor width per main point

// ========== Diagnostic logging ==========
#define USE_DIAG_LOG    1   // 1 = write CSV logs + console prints; 0 = silent (cloud test)

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
