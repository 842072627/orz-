// optsvr/MapTuner.h — 地图类型识别 + 自动设置 schedule_mode
#pragma once

#include "RuntimeConfig.h"
#include "MapProfile.h"
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>

namespace optsvr {

struct MapTuningState {
    bool loaded = false;
    int rows = 0;
    int cols = 0;
    int n_agents = 0;
    std::string map_name;
};

inline std::mutex& map_tuning_mutex() {
    static std::mutex m;
    return m;
}

inline MapTuningState& map_tuning_state() {
    static MapTuningState s;
    return s;
}

inline std::string map_tuning_target_name(const MapProfile& profile,
                                          const SharedEnvironment* env) {
    if (env != nullptr && !env->map_name.empty()) return env->map_name;
    return profile.map_name;
}

inline bool same_map_tuning_target(const MapTuningState& s,
                                   const MapProfile& profile,
                                   const SharedEnvironment* env) {
    return s.loaded &&
           s.rows == profile.rows &&
           s.cols == profile.cols &&
           s.n_agents == profile.n_agents &&
           s.map_name == map_tuning_target_name(profile, env);
}

inline void remember_map_tuning_target(MapTuningState& s,
                                       const MapProfile& profile,
                                       const SharedEnvironment* env) {
    s.loaded = true;
    s.rows = profile.rows;
    s.cols = profile.cols;
    s.n_agents = profile.n_agents;
    s.map_name = map_tuning_target_name(profile, env);
}

// 第一轮：按地图类型加载 Combined 赛道使用的完整 S/P/E 参数组合。
inline void load_for_combined(const MapProfile& profile, const SharedEnvironment* /*env*/) {
    auto& c = cfg();

    switch (profile.type) {

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 1;
            c.planner_mode = 11;  // test_useful FlowDecay copy
            c.ware_direct = true;

            c.executor_mode = 3;  // 3

            // PIBT history congestion: weak_band2, validated on warehouse_10000/7000
            // against history-off baseline. Keep it light so it mostly acts as a
            // local tie-break without increasing wait preference.

            // c.pibt_history_congestion_enabled = false;
            // c.pibt_history_window = 500;
            // c.pibt_history_free_threshold = 0.06f;
            // c.pibt_history_quad_threshold = 0.30f;
            // c.pibt_history_linear_weight = 0.30f;
            // c.pibt_history_quad_weight = 1.00f;
            // c.pibt_history_cap = 2.0f;
            // c.pibt_history_quad_power = 2.5f;
            // c.pibt_history_sort_mode = 2;
            // c.pibt_history_tie_base_eps = 2.0f;
            // c.pibt_history_apply_to_wait = true;
            // c.pibt_history_goal_exempt_dist = -1;

            //c.reassign_keep_bias = 14;
            //c.reassign_keep_bias = 5;
            //c.reassign_enabled = false;

            break;

        default:
            c.skip_test = true;
            break;
    }
}

inline void apply_map_tuning(const MapProfile& profile, const SharedEnvironment* env) {
    std::lock_guard<std::mutex> lock(map_tuning_mutex());
    auto& state = map_tuning_state();
    if (same_map_tuning_target(state, profile, env)) return;

    map_profile() = profile;
    auto& c = cfg();

    load_for_combined(profile, env);

    // 提交赛道切换点：默认 Combined。下面两个函数也按地图保留定制入口。
    // load_for_schedule(profile, env);  // Scheduler track
    // load_for_executor(profile, env);  // Executor track

    // 环境变量最高优先级：仅在设置时覆盖 MapTuner 的选择
    c.override_from_env();
    remember_map_tuning_target(state, profile, env);

    if (c.skip_test) {
        std::cerr << "[MapTuner] SKIP_TEST for: " << profile.type_to_string() << "\n";
    } else if (c.debug_level >= 1) {
        std::cerr << "[MapTuner] " << profile.type_to_string()
                  << " -> schedule_mode=" << c.schedule_mode
                  << " planner_mode=" << c.planner_mode
                  << " executor_mode=" << c.executor_mode << "\n";
    }
}

inline void load_map_tuning(const SharedEnvironment* env) {
    apply_map_tuning(analyze_map(env), env);
}

} // namespace optsvr
