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
#define FF_TH_BAND_1          1.52f   // threshold for width < 4 (very narrow)
#define FF_TH_BAND_2          1.52f   // threshold for width 4-12 (narrow)
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
#define USE_WALL_HUGGING        true    // master switch for all wall-hugging features
#define USE_WALL_HUGGING_PIBT    true    // PIBT successor tie-breaker wall-hugging
#define USE_WALL_HUGGING_ASTAR   true    // A* guide path branch-dist bias
#define WALL_HUG_WIDTH_MAX       20      // max corridor width (cells) for wall-hugging
#define WALL_HUG_BRANCH_BIAS_MAX 2       // max per-step branch bias for A* (0-5, larger = stronger)

// ========== Planner-layer lane flow gate (红绿灯) ==========
#define USE_PLANNER_LANE_FLOW   0   // 1=启用planner层lane方向门控
#define LANE_VOTE_PERIOD        80   // 每N tick投票翻转方向锁
#define LANE_PRIORITY_DELTA   1000  // 顺流agent优先级增加量
#define LANE_DIR_LOOKAHEAD      20  // 扫guide path前N步内的lane格求方向

// ========== Lane flow statistics ==========
#define USE_LANE_FLOW_STATS     1   // 1=统计forbids_move拦/放次数
#define LANE_CORE_CLEAR_THRESH  4   // 核心区人数<=此值视为清空
#define LANE_FLOW_RATIO_FLIP    3   // 拦/放流比>=此值强制翻锁
#define USE_LANE_FORBIDS_EXCLUDE_IDLE 1  // 1=无任务agent不受forbids_move限制

// ========== Dummy goal behavior for idle agents ==========
#define USE_DUMMY_GOAL_STAY     1   // 1=dummy goal = 当前位置
#define USE_DUMMY_EXIT_TARGET   1   // 1=没任务agent走到最近lane出口

// ========== Idle capacity control ==========
#define USE_IDLE_CAP         false   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

// ========== Migration flags ==========
#define USE_MP_CENTERLINE   1   // widest path centerline
#define USE_MP_DENSITY      1   // 1D main point density (heatmap)
#define USE_MP_WIDTH        1   // P10 corridor width per main point

// ========== Main zone direction lock hold (prevent lock release/flip when zone is hot) ==========
#define USE_MAIN_ZONE_LOCK_HOLD  true    // enable main zone density-based lock hold
#define MAIN_ZONE_HOLD_CAP       15      // zone density >= this → keep lock direction unchanged

// ========== Task segment scoring coefficients (统一评分系数) ==========
#define COEFF_AGENT_TASK1   1.0f    // agent→第一errand 距离系数
#define COEFF_TASK_SEGMENT  1.01f   // errand间 距离系数（task1→task2, task2→task3...）

// ========== Path heat check options ==========
#define EXCLUDE_AGENT_ZONE   false   // true=检查路径热区时跳过agent所在zone，false=全路径检查

// ========== Diagnostic logging ==========
#define USE_DIAG_LOG    0   // 1 = write CSV logs + console prints; 0 = silent (cloud test)

// ========== Congestion data collection (independent of lane flow gate) ==========
#define USE_CONGESTION_DATA  1   // 1 = collect per-tick core/lane congestion stats

// ========== Lane congestion relief (redirect counterflow + red light) ==========
#define LANE_RELIEF_ENABLED      1     // 拥塞疏导总开关
#define LANE_RELIEF_TICKS       300   // 疏导持续tick数
#define LANE_RELIEF_OCC_THRESH  75    // occ >= 此值触发条件1
#define LANE_RELIEF_S10_THRESH  2     // stuck10 >= 此值触发条件1
#define LANE_RELIEF_ALT_OCC     65    // occ >= 此值触发条件2(备选)
#define LANE_RELIEF_ALT_GROWTH  50    // 增长率 >= 此值触发条件2(备选)
#define LANE_RELIEF_BALANCE     0.3f  // 方向均衡度 >= 此值才允许触发
#define LANE_RELIEF_FRINGE_NEAR 5     // 距端点几步内的cell算"出口端"

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
