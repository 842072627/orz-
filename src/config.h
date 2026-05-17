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
#define ORZ_PLANNER     "my"       // ORZ planner: my PIBT (with g_agent_blocked check)
#define ORZ_EXECUTOR    "default"       // ORZ executor: default

// ========== Density cap ==========
#define ZONE_HARD_CAP  15  

#define ZONE_EXEMPT_PIBT_CAP  28       // 24x24 zone density cap, blocks when exceeded
// 24x24 zone density cap, blocks when exceeded

// ========== 1D main zone exemption ==========
#define EXEMPT_MAIN_THR  15      // main zone density below this exempts task from direction lock

// ========== Hard density limit ==========
#define USE_DENSITY_CAP_80  true   // enable hard density cap
#define DENSITY_CAP_80      100     // zones at this density block ALL tasks (regardless of direction)

// ========== Feature flags ==========
#define ORZ_PIBT_DIRECTION_BLOCK  true    // scheduler + PIBT direction block linkage

// ========== Hot zone exemption ==========
#define USE_HOT_ZONE_EXEMPT  true   // 热区豁免：热区agent不设block，PIBT正常执行

// ========== Idle capacity control (空载均匀空跑) ==========
#define USE_IDLE_CAP         true   // 空载容量控制开关
#define IDLE_RATIO           1.0f   // 正常时期空闲agent比例上限
#define IDLE_BOOT_TICKS      0    // 开局前N个tick用小比例控流
#define IDLE_BOOT_RATIO      0.4f   // boot期空闲agent比例上限

#define ORZ_DIRECTION_LOCK        true    // direction lock (lock_dir)

// ========== V4 Path-aligned square config ==========
#define SQUARE_SIZE     24      // 路径方块尺寸
#define SQUARE_STEP     16      // 采样步长
#define PREDICT_THRESHOLD 25    // 方块密度≥此值视为热区
#define PREDICT_HORIZON 8       // 预测前方N个方块
#define ADAPTIVE_RANGE  15      // damping range: density above threshold for 100% block (25+25=50)
#define HOT_SQUARE_EXEMPT 28    // 自身方块密度≥此值时豁免截停(让agent走出去)
#define DIR_LOCK_RADIUS 2       // 方向锁传播: 热方块+左右各2个共5个方块
#define DIR_LOCK_BOOT_TICKS 0  // 开局前N个tick默认方向锁从左到右

// Runtime strategy name getter (used by TaskScheduler.cpp)
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
