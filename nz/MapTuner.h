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
            c.schedule_mode = 8; 
            c.executor_mode = 6;  
            c.planner_mode  = 16; 
            c.precompute_heuristic = true;
            c.reassign_enabled = false;
            c.game_heuristic_hard_direction = true;
            c.game_idle_highway_relocate = true;
            c.game_skip_channel_errands = false;
            c.game_skip_longest_pct = 5;
            c.game_dir_flow_enabled = true;
            c.game_dir_flow_cap = 9999;
            c.game_dir_flow_gap = 0;
            c.game_dir_flow_decay = 0.9f;
            c.game_dir_flow_max_check_main = 1000;
            c.game_region_match = true;
            c.game_region_flow_batch = 64;
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
