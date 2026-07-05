// optsvr/MapTuner.h — 地图类型识别 + 自动设置 schedule_mode (fullfill only)
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

// 按地图类型加载参数（仅保留 warehouse/fullfill）
inline void load_for_combined(const MapProfile& profile, const SharedEnvironment* /*env*/) {
    auto& c = cfg();

    switch (profile.type) {

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 10;  // IronZone (含 future flow 方向锁)
            c.planner_mode = 11;   // TestUsefulFlowDecay
            c.ware_direct = true;
            c.dummy_goal_home = true;
            c.executor_mode = 3;   // DHExecutor

            // IronZone scheduler + future flow settings
            c.iron_zone_rows = 19;
            c.iron_zone_cols = 18;
            c.iron_max_radius = 1;
            c.iron_dist_weight = 6.0f;
            c.iron_task_len_weight = 1.25f;
            c.iron_long_task_th = 1250;
            c.iron_stats_interval = 10;
            c.iron_lane_enabled = 0;

            c.iron_future_flow_w = 1.0f;
            c.iron_future_flow_k = 1.0f;
            c.iron_future_flow_decay = 0.01f;
            c.iron_future_flow_lock_th = 200.0f;
            c.iron_future_flow_congest_cap = 200.0f;
            c.iron_future_flow_zone_rows = 40;
            c.iron_future_flow_zone_cols = 20;
            c.iron_ff_zone_tuning_enabled = true;
            c.iron_ff_edge_cols = 1;
            c.iron_ff_topbot_rows = 1;
            c.iron_ff_left_edge_congest_cap = 80.0f;
            c.iron_ff_right_edge_congest_cap = 200.0f;
            c.iron_ff_top_congest_cap = 200.0f;
            c.iron_ff_bottom_congest_cap = 200.0f;
            c.iron_ff_left_corner_penalty = 0.0f;
            c.iron_ff_right_corner_penalty = 0.0f;
            c.iron_ff_outer_penalty = 0.0f;
            c.iron_ff_left_edge_penalty = 0.0f;
            c.iron_ff_idle_w_dist = 2.0f;
            c.iron_ff_idle_w_repel = 10.0f;
            c.iron_ff_flow_block_pct = 0.95f;
            c.iron_ff_flow_lock_pct = 0.85f;
            break;

        default:
            c.skip_test = true;
            break;
    }
}

// 根据地图类型自动调整 RuntimeConfig
inline void apply_map_tuning(const MapProfile& profile, const SharedEnvironment* env) {
    std::lock_guard<std::mutex> lock(map_tuning_mutex());
    auto& state = map_tuning_state();
    if (same_map_tuning_target(state, profile, env)) return;

    map_profile() = profile;
    auto& c = cfg();
    load_for_combined(profile, env);

    // 环境变量最高优先级
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
