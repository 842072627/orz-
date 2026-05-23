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

// ========== Future flow direction lock (pure prediction, per-width thresholds) ==========
#define FUTURE_FLOW_LOCK_TH   1.72f    // baseline (only used when per-width config is disabled)
#define FUTURE_FLOW_LOCK_SPREAD 0       // lock radius: 0 = no propagation, each MP independent
#define FUTURE_FLOW_TH_MAX        2.8f    // max TH when active ratio = 0
#define FUTURE_FLOW_BRANCH_PENALTY  0.6f
#define FUTURE_FLOW_BRANCH_OFFSET   14

// ========== Per-width-zone direction lock thresholds ==========
// Width bands: [0, WIDTH_BAND_1)=no_lock, [WIDTH_BAND_1,WIDTH_BAND_2)=FF_TH_B1, etc.
// Width > WIDTH_BAND_4 = no lock (super wide, free flow)
#define USE_PER_WIDTH_LOCK    1       // 1=use per-width thresholds, 0=use fixed FUTURE_FLOW_LOCK_TH
#define WIDTH_BAND_1          4       // width < 4: very narrow
#define WIDTH_BAND_2          9       // width 4-8: narrow
#define WIDTH_BAND_3          16      // width 9-15: moderate
#define WIDTH_BAND_4          999     // width >= 16: wide (no lock)
#define FF_TH_BAND_1          1.2f    // threshold for width < 4 (very narrow, easy to lock)
#define FF_TH_BAND_2          1.72f   // threshold for width 4-8 (narrow)
#define FF_TH_BAND_3          2.2f    // threshold for width 9-15 (moderate)
// Width >= WIDTH_BAND_3: use TH_BAND_3 + active ratio relax or no lock
#define USE_BAND_3_RELAX      1       // apply active ratio relax to band 3

// ========== Branch/hot zone checks (uses zone24 density, all agents counted) ==========
#define ZONE_HARD_CAP  15
#define ZONE_EXEMPT_PIBT_CAP  15       // 24x24 zone density cap, blocks when exceeded

// ========== 1D main zone exemption ==========
#define EXEMPT_MAIN_THR  23      // kept for branch check fallback

// ========== Task segment distance limit ==========
#define MAX_TASK_SEGMENT_DIST  99999   // max main+branch distance for a single errand transition
#define GARBAGE_TASK_RATIO      0.05f  // bottom fraction of free tasks deferred each tick (0.0-1.0)

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

// ========== Active agent ratio lock relaxation (global) ==========
#define USE_ACTIVE_RATIO_RELAX   true    // global active-agent ratio relaxation
#define ACTIVE_RATIO_BASELINE    0.50f   // active ratio >= this  TH=1.7 (no relax)
#define ACTIVE_RATIO_RELAX_EXP  2       // curve exponent (1=linear, 2=quadratic, 3=cubic...)
#define ACTIVE_RATIO_DENSITY_MAX 30     // per-main-point density >= this  skip relaxation (zone still hot)
#define ACTIVE_RELAX_DELAY_TICKS 100     // ticks before active ratio relaxation kicks in

// ========== Idle capacity control ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

// ========== BFS distance table ==========
#define USE_BFS_TABLE       0   // 0: off-main组件 (本地), 1: 全量BFS距离矩阵 (云端)

// ========== Migration flags (step-by-step control) ==========
#define USE_MP_CENTERLINE   1   // Phase 1: A* widest path centerline
#define USE_MP_DENSITY      1   // Phase 2: 1D main point density
#define USE_MP_WIDTH        1   // corridor width-adaptive direction lock thresholds
#define WIDTH_WIDE_THR      12  // width > this → WIDE (+WIDTH_WIDE_BONUS, legacy)
#define WIDTH_SUPER_THR     18  // width > this → SUPER WIDE (+WIDTH_SUPER_BONUS, legacy)
#define WIDTH_WIDE_BONUS    2.0f  // extra TH for wide corridor (legacy, unused with USE_PER_WIDTH_LOCK)
#define WIDTH_SUPER_BONUS   4.0f  // extra TH for super wide corridor (legacy, unused with USE_PER_WIDTH_LOCK)

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }










