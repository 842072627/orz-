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
#define FUTURE_FLOW_LOCK_TH   1.72f    // future flow >= this → lock direction
#define FUTURE_FLOW_LOCK_SPREAD 2       // lock radius: ±N main points
#define FUTURE_FLOW_TH_MAX        2.8f    // max TH when active ratio = 0
#define FUTURE_FLOW_RELAX_DELAY_TICKS 100  // ticks before density relaxation kicks in (DEPRECATED, use ACTIVE_RELAX_DELAY_TICKS)
#define FUTURE_FLOW_BRANCH_PENALTY  0.6f
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
#define ACTIVE_RATIO_BASELINE    0.50f   // active ratio >= this  TH=1.7 (no relax)
#define ACTIVE_RATIO_RELAX_EXP  2       // curve exponent (1=linear, 2=quadratic, 3=cubic...)
#define ACTIVE_RATIO_DENSITY_MAX 30     // per-main-point density >= this  skip relaxation (zone still hot)
#define ACTIVE_RELAX_DELAY_TICKS 100     // ticks before active ratio relaxation kicks in

// ========== Idle capacity control (空载均匀空跑) ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

// ========== BFS distance table (0=offmain only, 1=full matrix for cloud) ==========
#define USE_BFS_TABLE       1   // 0: off-main only (本地), 1: 全量矩阵 (云端)

// ========== Migration flags (step-by-step control, can revert individually) ==========
#define USE_MP_CENTERLINE   1   // Phase 1: A* widest path centerline (matches heatmap analysis)
#define USE_MP_DENSITY      1   // Phase 2: 1D main point density (24x24 squares centered on main points)
#define USE_MP_WIDTH        1   // P10 width gradient → direct flow threshold
#define WIDTH_GRAD_1        12   // width < this → TH_GRAD_1 (极窄)
#define WIDTH_GRAD_2        18   // width < this → TH_GRAD_2 (窄, baseline)
#define WIDTH_GRAD_3        999  // width < this → TH_GRAD_3 (中)
// width >= WIDTH_GRAD_3 → TH_GRAD_4 (宽不锁)
#define TH_GRAD_1           1.52f
#define TH_GRAD_2           3.72f
#define TH_GRAD_3           5.72f
#define TH_GRAD_4           100.72f   // 宽走廊不锁

// ========== Tidal Refuge: 逆流贴边避难 ==========
#define USE_TIDAL_REFUGE        1   // 0=off, 1=on
#define USE_LANE_NARROW_BOTTLENECK 1 // 1=用lane zone识别窄瓶颈, 0=用p10_width<=TIDAL_WIDTH_TH
#define TIDAL_WIDTH_TH          5   // p10_width <= 此值视为窄瓶颈 (USE_LANE_NARROW_BOTTLENECK=0时生效)
#define TIDAL_SEARCH_MARGIN     3   // 候选格搜索范围 = 瓶颈长度 × 此值
#define TIDAL_REFUGE_MAX_DIST   5   // agent距硬编码lane入口≤此main_dist时触发避难reroute

// ========== Zone heat refuge control (热度疏导阈值) ==========
#define ZONE_REFUGE_HEAT_ON     35   // zone density >= 此值且上升 → 启用避难疏导
#define ZONE_REFUGE_HEAT_OFF    25   // zone density < 此值 → 关闭避难疏导

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }










