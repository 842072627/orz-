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
#define ORZ_SCHEDULER   "heat_v7"       // ORZ调度器: V7区域交错
#define ORZ_PLANNER     "default"       // ORZ规划器: 默认PIBT（待定制）
#define ORZ_EXECUTOR    "default"       // ORZ执行器: 默认（待定制）
