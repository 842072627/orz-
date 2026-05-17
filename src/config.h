// config.h - Combined Track global configuration
// All map strategy selection controlled here
// v5: Edge lane pushout architecture (center conveyor + edge emergency lane)

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
#define ORZ_SCHEDULER   "centerline"    // ORZ scheduler: centerline heatmap + greedy match
#define ORZ_PLANNER     "my"            // ORZ planner: my PIBT
#define ORZ_EXECUTOR    "default"       // ORZ executor: default

// ========== Core direction lock config ==========
#define ZONE_HARD_CAP          15       // 方块密度>=此值才会形成方向锁
#define DIR_LOCK_RADIUS         2       // 方向锁传播: 热方块+左右各2个共5个方块
#define DIR_LOCK_BOOT_TICKS    0       // 开局前N个tick默认方向锁从左到右

// ========== Edge lane pushout (全新架构!) ==========
#define USE_COLD_ZONE_PUSHOUT   true    // 开！（实际使用新的edge lane pushout）
// 管控热度阈值
#define CONTROL_M_THRESHOLD     5       // M zone heat≥5（agents/zone）管控
#define CONTROL_W_THRESHOLD     10      // W zone heat≥10（agents/zone）管控
// 死区保护
#define ENABLE_DEAD_END_PROTECTION true // 方向锁传播避开死角zone

// ========== Feature flags ==========
#define ORZ_DIRECTION_LOCK           true   // direction lock (lock_dir)
#define ORZ_PIBT_DIRECTION_BLOCK     true   // scheduler + PIBT direction block linkage
#define PIBT_RECURSION_DEPTH         50     // causalMY_PIBT递归深度限制

// ========== V4 Path-aligned square config ==========
#define SQUARE_SIZE     24      // 路径方块尺寸
#define SQUARE_STEP     16      // 采样步长

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
