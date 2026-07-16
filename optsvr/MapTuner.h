#pragma once

#include "MapProfile.h"
#include "RuntimeConfig.h"

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
    static std::mutex mutex;
    return mutex;
}

inline MapTuningState& map_tuning_state() {
    static MapTuningState state;
    return state;
}

inline std::string tuning_map_name(const MapProfile& profile,
                                   const SharedEnvironment* env) {
    return env && !env->map_name.empty() ? env->map_name : profile.map_name;
}

inline bool same_tuning_target(const MapTuningState& state,
                               const MapProfile& profile,
                               const SharedEnvironment* env) {
    return state.loaded && state.rows == profile.rows &&
           state.cols == profile.cols && state.n_agents == profile.n_agents &&
           state.map_name == tuning_map_name(profile, env);
}

inline void apply_game_tuning(RuntimeConfig& config) {
    // Keep this block aligned with MAIN2026/optsvr/MapTuner.h GAME_DOMAIN.
    config.skip_test = false;
    config.schedule_mode = 8;
    config.executor_mode = 11;
    config.planner_mode = 16;
    config.precompute_heuristic = true;
    config.game_platoon_scope = 3;
    config.game_guide_path_enabled = false;
    config.dist2path_prewarm_enabled = true;
    config.dist2path_prewarm_radius = 3;
    config.dist2path_prewarm_threads = 0;
    config.dist2path_prewarm_min_left_ms = 50;
    config.game_highway_enabled = true;
    config.game_heuristic_hard_direction = true;
    config.game_idle_highway_relocate = true;
    config.game_skip_channel_errands = false;
    config.game_bal_assign_ratio = 1.00f;
    config.game_dir_flow_enabled = true;
    config.game_dir_flow_hard_filter = true;
    config.game_dir_flow_cap = 300;
    config.game_dir_flow_gap = 0;
    config.game_dir_flow_decay = 0.95f;
    config.game_dir_flow_zone_horizon = 0;
    config.game_dir_flow_horizon_exempt_ratio = 0.2f;
    config.game_dir_flow_dynamic_cap = true;
    config.game_dir_flow_dynamic_cap_window = 500;
    config.game_dir_flow_dynamic_cap_interval = 200;
    config.game_dir_flow_dynamic_cap_low = 0.005f;      // 0.01
    config.game_dir_flow_dynamic_cap_high = 0.05f;      // 0.03
    config.game_dir_flow_dynamic_cap_inc = 5.0f;
    config.game_dir_flow_dynamic_cap_dec = 0.95f;
    config.game_dir_flow_dynamic_cap_min_ratio = 0.20f;     // 0.5
    config.game_dir_flow_dynamic_cap_max_ratio = 2.00f;     // 1.4
    config.game_dir_flow_random_shortest = true;
    config.game_score_exempt_enabled = true;
    config.game_exempt_main = 23;       // 仅 score_exempt=true 时生效
    config.game_distance_score_mode = 3;
    config.game_pickup_weight = 2.0f;
    config.game_region_match = true;
    config.game_region_flow_batch = 64;
    config.game_search_radius = 3;
    config.game_global_max_task_scan = 50;
    config.game_global_cand_topk = 32;
    config.game_flow_soft_over_cap_growth = 1.20f;
    config.game_short_task_throttle = false;
    config.game_short_task_test_ticks = 40000;
    config.game_short_task_final_open_ticks = 12500;
    config.game_short_task_release_ratio = 0.25f;
    config.game_reassign = false;
    config.game_lns_pct = 5;
    config.game_keep_bias = 10.0f;
    config.game_min_dist = 10;
    config.game_idle_global_match = false;
    config.game_idle_global_ticks = 500;
    config.game_idle_global_max_agents = 64;
}

inline void load_map_tuning(const SharedEnvironment* env) {
    const MapProfile profile = analyze_map(env);
    std::lock_guard<std::mutex> lock(map_tuning_mutex());
    auto& state = map_tuning_state();
    if (same_tuning_target(state, profile, env)) return;

    map_profile() = profile;
    auto& config = cfg();
    config.skip_test = true;
    if (profile.type == MapType::GAME_DOMAIN) apply_game_tuning(config);

    // As in MAIN2026, explicitly supplied environment values take precedence.
    config.override_from_env();

    state.loaded = true;
    state.rows = profile.rows;
    state.cols = profile.cols;
    state.n_agents = profile.n_agents;
    state.map_name = tuning_map_name(profile, env);

    if (config.skip_test) {
        std::cerr << "[MapTuner] SKIP_TEST for: "
                  << profile.type_to_string() << "\n";
    } else if (config.debug_level >= 1) {
        std::cerr << "[MapTuner] GAME_DOMAIN -> schedule_mode=8"
                  << " planner_mode=16 executor_mode=11\n";
    }
}

}  // namespace optsvr
