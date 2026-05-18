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

// ========== Density cap ==========
#define ZONE_HARD_CAP  15  

#define ZONE_EXEMPT_PIBT_CAP  15       // 24x24 zone density cap, blocks when exceeded
// 24x24 zone density cap, blocks when exceeded

// ========== 1D main zone exemption ==========
#define EXEMPT_MAIN_THR  23      // main zone density below this exempts task from direction lock

// ========== Hard density limit ==========
#define USE_DENSITY_CAP_80  true   // enable hard density cap
#define DENSITY_CAP_80      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  true    // scheduler + PIBT direction block linkage

// ========== Hot zone exemption ==========
#define USE_HOT_ZONE_EXEMPT  true   // 热区豁免：热区agent不设block，PIBT正常执行

// ========== Idle capacity control (空载均匀空跑) ==========
#define USE_IDLE_CAP         true   // 空载容量控制开关
#define IDLE_RATIO           0.4f  // 最大活跃 agent 比例 (0~1.0)

#define ORZ_DIRECTION_LOCK        true    // direction lock (lock_dir)

// ========== Migration flags (step-by-step control, can revert individually) ==========
#define USE_MP_CENTERLINE   1   // Phase 1: A* widest path centerline (matches heatmap analysis)
#define USE_MP_DENSITY      1   // Phase 2: 1D main point density (24x24 squares centered on main points)
#define USE_MP_DIRECTION    1   // Phase 3: 1D direction lock based on main point density

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
