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

// ========== Future flow direction lock (pure prediction, no density) ==========
#define FUTURE_FLOW_LOCK_TH   1.6f    // future flow >= this → lock direction
#define FUTURE_FLOW_LOCK_SPREAD 2       // lock radius: ±N main points
#define FUTURE_FLOW_TH_MAX        2.0f    // max TH when active ratio = 0
#define FUTURE_FLOW_RELAX_DELAY_TICKS 100  // ticks before density relaxation kicks in (DEPRECATED, use ACTIVE_RELAX_DELAY_TICKS)
#define FUTURE_FLOW_BRANCH_PENALTY  0.8f

#define FUTURE_FLOW_BRANCH_OFFSET   14
// ========== Branch/hot zone checks (uses zone24 density, all agents counted) ==========
#define ZONE_HARD_CAP  15  

#define ZONE_EXEMPT_PIBT_CAP  15       // 24x24 zone density cap, blocks when exceeded

// ========== 1D main zone exemption (replaced by future-flow exempt) ==========
#define EXEMPT_MAIN_THR  23      // kept for branch check fallback

// ========== Task segment distance limit ==========
#define MAX_TASK_SEGMENT_DIST  99999   // max main+branch distance for a single errand transition
#define GARBAGE_TASK_RATIO      0.05f // bottom fraction of free tasks deferred each tick (0.0-1.0)

// ========== Hard density limit ==========
#define USE_DENSITY_CAP_80  true   // enable hard density cap
#define DENSITY_CAP_80      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  true    // scheduler + PIBT direction block linkage

// ========== Density exclusion ==========
#define USE_DENSITY_EXCLUDE_IDLE  false    // 排除无任务/被截停agent的密度贡献

// ========== Hot zone exemption ==========
#define USE_HOT_ZONE_EXEMPT  true   // 热区豁免：热区agent不设block，PIBT正常执行

// ========== Wall-hugging in corridors (PIBT tie-breaker + A* guide path) ==========
#define USE_WALL_HUGGING        true    // master switch for all wall-hugging features
#define USE_WALL_HUGGING_PIBT    true    // PIBT successor tie-breaker wall-hugging
#define USE_WALL_HUGGING_ASTAR   true    // A* guide path branch-dist bias
#define WALL_HUG_WIDTH_MAX       20      // max corridor width (cells) for wall-hugging
#define WALL_HUG_BRANCH_BIAS_MAX 2       // max per-step branch bias for A* (0-5, larger = stronger)

// ========== Active agent ratio lock relaxation (global, replaces per-main-point density relax) ==========
#define USE_ACTIVE_RATIO_RELAX   true    // global active-agent ratio relaxation
#define ACTIVE_RATIO_BASELINE    0.45f   // active ratio >= this  TH=1.7 (no relax)
#define ACTIVE_RATIO_RELAX_EXP  1       // curve exponent (1=linear, 2=quadratic, 3=cubic...)
#define ACTIVE_RATIO_DENSITY_MAX 30     // per-main-point density >= this  skip relaxation (zone still hot)
#define ACTIVE_RELAX_DELAY_TICKS 50     // ticks before active ratio relaxation kicks in

// ========== Idle capacity control (空载均匀空跑) ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           1.0f  // 最大活跃 agent 比例 (0~1.0)

// ========== Migration flags (step-by-step control, can revert individually) ==========
#define USE_MP_CENTERLINE   1   // Phase 1: A* widest path centerline (matches heatmap analysis)
#define USE_MP_DENSITY      1   // Phase 2: 1D main point density (24x24 squares centered on main points)
#define USE_MP_WIDTH        1   // corridor width-adaptive direction lock thresholds
#define WIDTH_WIDE_THR      12  // width > this → WIDE (+WIDTH_WIDE_BONUS)
#define WIDTH_SUPER_THR     18  // width > this → SUPER WIDE (+WIDTH_SUPER_BONUS)
#define WIDTH_WIDE_BONUS    1.0f  // extra TH for wide corridor
#define WIDTH_SUPER_BONUS   2.0f  // extra TH for super wide corridor

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }










