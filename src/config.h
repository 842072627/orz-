// config.h — Combined Track 全局配置
// 所有地图策略选择在此集中控制

#pragma once
#include <string>
#include "SharedEnv.h"

// ========== 地图识别 ==========
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
    return is_ORZ(env);  // 只有ORZ有定制策略
}

// ========== 策略选择 ==========
#define ORZ_SCHEDULER   "centerline"    // ORZ调度器: centerline=中心线热力图+贪心匹配 (v2)
#define ORZ_PLANNER     "default"       // ORZ规划器: 默认PIBT（待定制）
#define ORZ_EXECUTOR    "default"       // ORZ执行器: 默认（待定制）

// 运行时获取策略名（用于TaskScheduler.cpp）
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }
