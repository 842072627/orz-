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
#define FUTURE_FLOW_LOCK_SPREAD 0       // lock radius: 0 = no propagation, each MP independent
#define FUTURE_FLOW_BRANCH_PENALTY  0.6f
#define FUTURE_FLOW_BRANCH_OFFSET   14

// ========== Per-width-zone direction lock thresholds ==========
#define WIDTH_BAND_1          4       // width < 4: very narrow
#define WIDTH_BAND_2          12       // width 4-8: narrow
#define WIDTH_BAND_3          18      // width 9-15: moderate
#define FF_TH_BAND_1          1.42f   // threshold for width < 4 (very narrow)
#define FF_TH_BAND_2          1.42f   // threshold for width 4-12 (narrow)
#define FF_TH_BAND_3          3.52f    // threshold for width 12-18 (moderate)
#define FF_TH_BAND_4          5.72f // threshold for width >= 16 (wide, never lock by default)

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
#define GARBAGE_TASK_RATIO      0.05f  // bottom fraction of free tasks deferred each tick (0.0-1.0)

// ========== Hard density limit ==========
#define USE_DENSITY_HARD_CAP  true    // enable hard density cap
#define DENSITY_HARD_CAP      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  false    // scheduler + PIBT direction block linkage

// ========== Density exclusion ==========
#define USE_DENSITY_EXCLUDE_IDLE  false    // 排除无任务/被截停agent的密度贡献

// ========== Wall-hugging in corridors (PIBT tie-breaker + A* guide path) ==========
#define USE_WALL_HUGGING        false   // master switch for all wall-hugging features
#define USE_WALL_HUGGING_PIBT    false   // PIBT successor tie-breaker wall-hugging
#define USE_WALL_HUGGING_ASTAR   false   // A* guide path branch-dist bias
#define WALL_HUG_WIDTH_MAX       20      // max corridor width (cells) for wall-hugging
#define WALL_HUG_BRANCH_BIAS_MAX 2       // max per-step branch bias for A* (0-5, larger = stronger)

// ========== Tidal Refuge: 逆流贴边避难 ==========
#define USE_TIDAL_REFUGE        1   // 0=off, 1=on
#define TIDAL_REFUGE_MAX_DIST   5   // agent距硬编码lane入口≤此main_dist时触发避难reroute
#define TIDAL_REFUGE_ZONE_EXPAND     10  // 窄道main_dist范围外扩多少视为控制区
#define MICRO_VOTE_PERIOD       80  // 微观锁投票周期（tick）
#define MICRO_HYSTERESIS        2.0f // 微观锁翻转阈值倍数
#define REFUGE_PIBT_PENALTY     0   // refuge agent PIBT减权(0=关闭, -1000=冠军planner模式)

// ========== Debug ==========
#define TIDAL_REFUGE_DEBUG_INTERVAL 500  // 每多少tick打印一次debug
#define TIDAL_AGENT_DEBUG_MAX       20   // debug最多显示多少个避难agent

// ========== A* 启发式 ==========
#define LARGE_MAP_CELL_THRESHOLD    90000

// ========== Idle capacity control ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

// ========== Migration flags ==========
#define USE_MP_CENTERLINE   1   // widest path centerline
#define USE_MP_DENSITY      1   // 1D main point density (heatmap)
#define USE_MP_WIDTH        1   // P10 corridor width per main point

// ========== Main zone direction lock hold (prevent lock release/flip when zone is hot) ==========
#define USE_MAIN_ZONE_LOCK_HOLD  false    // enable main zone density-based lock hold
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
