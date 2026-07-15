// optsvr/MapTuner.h — GAME_DOMAIN only; other map types default to skip
#pragma once

#include "RuntimeConfig.h"
#include "MapProfile.h"
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>

namespace nz {

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
                                          SharedEnvironment* env) {
    if (env != nullptr && !env->map_name.empty()) return env->map_name;
    return profile.map_name;
}

inline bool same_map_tuning_target(const MapTuningState& s,
                                   const MapProfile& profile,
                                   SharedEnvironment* env) {
    return s.loaded &&
           s.rows == profile.rows &&
           s.cols == profile.cols &&
           s.n_agents == profile.num_agents &&
           s.map_name == map_tuning_target_name(profile, env);
}

inline void remember_map_tuning_target(MapTuningState& s,
                                       const MapProfile& profile,
                                       SharedEnvironment* env) {
    s.loaded = true;
    s.rows = profile.rows;
    s.cols = profile.cols;
    s.n_agents = profile.num_agents;
    s.map_name = map_tuning_target_name(profile, env);
}

inline void load_for_combined(const MapProfile& profile, const SharedEnvironment*) {
    auto& c = cfg();

    switch (profile.type) {

        case MapType::GAME_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 8;  // GameBalanceOptScheduler
            c.executor_mode = 6;  // GameExecutor
            c.planner_mode  = 16; // GameDirectionalPlanner
            c.precompute_heuristic = true;

            c.game_heuristic_hard_direction = true;
            c.game_idle_highway_relocate = true;

            // optimizer 预置参数
            c.dist2path_prewarm_enabled = true;
            c.game_skip_channel_errands = false;

            c.game_bal_assign_ratio = 1.0f;
            c.game_highway_enabled = true;
            c.game_guide_path_enabled = false; // 关掉 guide 更新加速
            c.game_dir_flow_enabled = true;
            c.game_dir_flow_hard_filter = true;
            c.game_dir_flow_cap = 300;
            c.game_dir_flow_gap = 0;
            c.game_dir_flow_decay = 0.95f;
            c.game_dir_flow_zone_horizon = 0;
            c.game_dir_flow_horizon_exempt_ratio = 0.2f;
            c.game_dir_flow_dynamic_cap = true;
            c.game_dir_flow_dynamic_cap_window = 500;
            c.game_dir_flow_dynamic_cap_interval = 200;
            c.game_dir_flow_dynamic_cap_low = 0.01f;
            c.game_dir_flow_dynamic_cap_high = 0.03f;
            c.game_dir_flow_dynamic_cap_inc = 5.0f;
            c.game_dir_flow_dynamic_cap_dec = 0.95f;
            c.game_dir_flow_dynamic_cap_min_ratio = 0.50f;
            c.game_dir_flow_target_ratio = 0.69f; // 低拥塞
            c.game_dir_flow_dynamic_cap_max_ratio = 1.40f;
            c.game_dir_flow_random_shortest = true;

            c.game_score_exempt_enabled = false;
            c.game_distance_score_mode = 3;
            c.game_pickup_weight = 2.0f;
            c.game_region_match = true;
            c.game_region_flow_batch = 64;
            c.game_search_radius = 5;
            c.game_sched_time_ratio = 1.0f;  // scheduler 不限制时间
            c.game_main_search_radius = 500;
            c.game_sign_filter = 1;
            c.game_global_max_task_scan = 50;
            c.game_global_cand_topk = 32;
            c.game_flow_soft_over_cap_growth = 1.20f;

            c.game_short_task_throttle = false;
            c.game_short_task_test_ticks = 40000;
            c.game_short_task_final_open_ticks = 12500;
            c.game_short_task_release_ratio = 0.25f;

            c.game_reassign = true;
            c.game_lns_pct = 5;
            c.game_keep_bias = 10.0f;
            c.game_min_dist = 10;

            c.game_idle_global_match = false;
            c.game_idle_global_ticks = 500;
            c.game_idle_global_max_agents = 64;
            break;

        default:
            // 非 ORZ 地图默认 skip，不执行任何调度/规划
            c.skip_test = true;
            c.schedule_mode = 0;
            c.executor_mode = 0;
            c.planner_mode = 0;
            break;
    }
}

inline void apply_map_tuning(const MapProfile& profile, SharedEnvironment* env) {
    std::lock_guard<std::mutex> lock(map_tuning_mutex());
    auto& state = map_tuning_state();
    if (same_map_tuning_target(state, profile, env)) return;
    map_profile() = profile;
    load_for_combined(profile, env);
    auto& c = cfg();
    c.override_from_env();
    remember_map_tuning_target(state, profile, env);
}

inline void load_map_tuning(SharedEnvironment* env) {
    MapProfile profile;
    analyze_map(env, env->num_of_agents, 0);
    profile = map_profile();
    apply_map_tuning(profile, env);
}

} // namespace nz
