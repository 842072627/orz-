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

// ========== BFS distance table ==========
#define USE_BFS_TABLE       1   // 0: off-main only (本地), 1: 全量矩阵 (云端)

// ========== Future flow direction lock (pure prediction, per-width thresholds) ==========
#define FUTURE_FLOW_LOCK_TH   1.72f    // baseline threshold for wide corridors
#define FUTURE_FLOW_LOCK_SPREAD 2       // lock radius: 0 = no propagation, each MP independent
#define FUTURE_FLOW_BRANCH_PENALTY  0.6f
#define FUTURE_FLOW_BRANCH_OFFSET   14

// ========== Per-width-zone direction lock thresholds ==========
#define WIDTH_BAND_1          4       // width < 4: very narrow
#define WIDTH_BAND_2          12       // width 4-8: narrow
#define WIDTH_BAND_3          18      // width 9-15: moderate
#define FF_TH_BAND_1          9999.0f   // threshold for width < 4 (very narrow)
#define FF_TH_BAND_2          9999.0f   // threshold for width 4-12 (narrow)
#define FF_TH_BAND_3          9999.0f    // threshold for width 12-18 (moderate)
#define FF_TH_BAND_4          9999.0f // threshold for width >= 16 (wide, never lock by default)

// ========== Active agent ratio lock relaxation ==========
#define USE_ACTIVE_RATIO_RELAX   true
#define FUTURE_FLOW_TH_MAX        2.5f
#define FUTURE_FLOW_RELAX_DELAY_TICKS 100
#define ACTIVE_RATIO_BASELINE    0.50f
#define ACTIVE_RATIO_RELAX_EXP  2
#define ACTIVE_RATIO_DENSITY_MAX 30
#define ACTIVE_RELAX_DELAY_TICKS 100

// ========== Task segment distance limit ==========
#define MAX_TASK_SEGMENT_DIST  99999   // max main+branch distance for a single errand transition
#define GARBAGE_TASK_RATIO      0.00f  // bottom fraction of free tasks deferred each tick (0.0-1.0)

// ========== Hard density limit ==========
#define USE_DENSITY_HARD_CAP  true    // enable hard density cap
#define DENSITY_HARD_CAP      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  false
#define ORZ_HIGHWAY_FORCE_MOVE     true // 3方向封禁时强制agent走唯一通道(默认关)    // scheduler + PIBT direction block linkage
#define ORZ_HIGHWAY_IDLE_EXIT    true // idle agent on labeled cell -> BFS to nearest unlabeled cell
#define ORZ_HIGHWAY_IDLE_FOLLOW  true // idle agent on forbid cell -> follow highway direction to exit

// ========== Density exclusion ==========
#define USE_DENSITY_EXCLUDE_IDLE  false    // 排除无任务/被截停agent的密度贡献

// ========== Wall-hugging in corridors (PIBT tie-breaker + A* guide path) ==========
#define USE_WALL_HUGGING        false    // master switch for all wall-hugging features
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

// ========== Main zone direction lock hold (prevent lock release/flip when zone is hot) ==========
#define USE_MAIN_ZONE_LOCK_HOLD  true    // enable main zone density-based lock hold
#define MAIN_ZONE_HOLD_CAP       16      // zone density >= this → keep lock direction unchanged

// ========== Task segment scoring coefficients (统一评分系数) ==========
#define COEFF_AGENT_TASK1   1.0f    // agent→第一errand 距离系数
#define COEFF_TASK_SEGMENT  1.01f   // errand间 距离系数（task1→task2, task2→task3...）

// ========== Path heat check options ==========
#define EXCLUDE_AGENT_ZONE   false   // true=检查路径热区时跳过agent所在zone，false=全路径检查

// ========== Diagnostic logging ==========
#define USE_DIAG_LOG    0   // 1 = write CSV logs + console prints; 0 = silent (cloud test)

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
