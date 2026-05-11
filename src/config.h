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
#define ZONE_HARD_CAP  15       // 24x24 zone density cap, blocks when exceeded

// ========== 1D main zone exemption ==========
#define EXEMPT_MAIN_THR  20      // main zone density below this exempts task from direction lock

// ========== Hard density limit ==========
#define USE_DENSITY_CAP_80  true   // enable hard density cap
#define DENSITY_CAP_80      40     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  true    // scheduler + PIBT direction block linkage
#define ORZ_DIRECTION_LOCK        true    // direction lock (lock_dir)

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
