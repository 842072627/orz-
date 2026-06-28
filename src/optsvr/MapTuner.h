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

        case MapType::MAZE_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 9;    // 9
            c.planner_mode = 8;

            c.executor_mode = 7;    // 3

            // c.p1_flow_vertex_penalty_enabled = false;
            // c.flow_vertex_cost_mode = 4;
            // c.flow_vertex_cost_threshold = 0;
            // c.flow_vertex_cost_weight = 10;
            // c.flow_vertex_cost_cap = 100;
            // c.flow_vertex_cost_exp_base = 3.0f;
            // c.flow_vertex_cost_linear_end = 4;
            // c.flow_vertex_sens_decay = 0.92f;
            // c.flow_vertex_base_weight = 80;

            // c.flow_decay = 1.0;     // no flow decay

            //c.heap_dist_decay_factor = 0.4;
            //c.heap_dist_decay_interval_steps = 3000;

            break;

        case MapType::ROOM_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 7;    // OptRoomScheduler
            c.executor_mode = 5;    // 5 RoomExecutor
            c.planner_mode = 7;     // RoomTidalPlanner (潮汐门控)
            // 最优参数 (R47-IDEA6 验证: avg=742.3, peak=756)
            c.opt_room_assign_ratio   = 0.8f;  // R44~R49验证: 0.75稳定最优
            c.opt_room_time_safety    = 1.2f;   // 默认安全系数
            c.opt_room_match_mode     = 1;      // greedy匹配
            c.opt_room_dist_first_w   = 2.0f;
            // TidalPlanner 最优参数 (R47-IDEA6: offpath15+bt60+step12)
            c.tidal_period            = 60;     // R44/R48验证: 60最优
            c.tidal_off_path          = 15;     // R48验证: 15是甜蜜点(10/20均差)
            c.tidal_backtrack_penalty = 60;     // R47/R48验证: bt60 avg最高
            c.tidal_backtrack_step    = 12;     // 配合bt60
            c.tidal_guide_bonus       = 30;     // 默认值(bonus20/40在r075下负面)
            c.tidal_wait_penalty      = 12;     // 默认值
            c.tidal_forward_skip      = 4;      // 默认值(fskip=0负面)
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 10;  // IronZone (含 future flow 方向锁)
            c.planner_mode = 11;   // test_useful FlowDecay copy
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

            // IronZone scheduler + future flow settings
            c.iron_zone_rows = 19;
            c.iron_zone_cols = 18;
            c.iron_max_radius = 1;
            c.iron_dist_weight = 1.8f;
            c.iron_task_len_weight = 1.25f;
            c.iron_long_task_th = 1250;
            c.iron_stats_interval = 10;
            c.iron_lane_enabled = 0;

            c.iron_future_flow_w = 1.0f;
            c.iron_future_flow_k = 1.0f;
            c.iron_future_flow_decay = 0.01f;
            c.iron_future_flow_lock_th = 100.0f;  // 方向锁阈值
            c.iron_future_flow_congest_cap = 320.0f;
            c.iron_future_flow_zone_rows = 20;
            c.iron_future_flow_zone_cols = 14;

            break;

        case MapType::CITY_DOMAIN:
            c.skip_test = true;
            c.planner_mode = 1;
            c.schedule_mode = 1;
            c.executor_mode = 7;

            c.heap_dist_decay_factor = 0.3;
            c.heap_dist_decay_interval_steps = 7000;
            break;

        case MapType::GAME_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 8;  // GameBalanceOptScheduler (中心线调度)
            c.executor_mode = 6;  // 6
            c.planner_mode  = 2;  // GameOptPlanner (FlowDecay + PibtLock)

            c.reassign_enabled = false;
            // 已验证最优参数
            c.game_width_aware       = true;   // 瓶颈宽度感知容量控制
            c.game_narrow_threshold  = 4;      // 宽度<=4视为窄瓶颈
            c.game_narrow_cap_ratio  = 0.5f;   // 窄瓶颈容量 = width * 0.5
            c.game_pibt_counter_flow = true;   // PIBT逆流惩罚
            c.game_counter_flow_cost = 20;     // 逆流额外代价
            c.game_urgency_priority  = true;   // 紧迫度优先级
            c.game_urgency_weight    = 10.0f;  // urgency weight
            c.game_urgency_threshold = 50;     // urgency 阈值
            c.game_search_radius     = 1;      // 默认搜索半径
            c.game_late_search_radius = 3;     // 后期搜索半径
            c.game_late_phase_ratio  = 0.4f;   // 40%后进入后期模式
            break;

        case MapType::IRON_HARVEST_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 10;             // IronZoneScheduler from IDEA/IRON
            c.executor_mode = 0;              // 0
            c.planner_mode = 12;              // IronFlowDecay wrapper for IDEA/IRON planner

            c.reassign_enabled = false;

            // Keep the validated IRON line self-contained in MapTuner.
            // The original IDEA/IRON planner is mode=1 in that snapshot; in MAIN2026
            // the migrated class is exposed as mode=12.
            // c.precompute_heuristic = false;
            c.dist2path_prewarm_enabled = false;
            c.dist2path_prewarm_radius = 3;
            c.dist2path_prewarm_threads = 0;
            c.dist2path_prewarm_min_left_ms = 50;

            // Match IDEA/IRON's active IRON defaults for the pieces present in MAIN2026.
            c.iron_zone_rows = 19;
            c.iron_zone_cols = 18;
            c.iron_max_radius = 1;
            c.iron_dist_weight = 1.8f;
            c.iron_task_len_weight = 1.25f;
            c.iron_long_task_th = 1250;
            c.iron_stats_interval = 10;
            c.iron_lane_enabled = 0;
            c.iron_lane_guard_dist = 20;
            c.iron_lane_core_flip_th = 10;
            c.iron_lane_flow_ratio_flip = 2;
            c.iron_lane_vote_period = 80;
            c.iron_lane_idle_bonus = 0.5f;
            c.iron_lane_debug = 0;
            c.iron_lane_lru_sort = 1;
            c.iron_lane_outside_vote = 1;
            c.iron_lane_exit_buffer_radius = 3;
            c.iron_lane_exit_priority = 1000.0f;
            c.iron_lane_buffer_penalty = 1000.0f;

            // flow decay
            c.iron_future_flow_w = 1.0f;
            c.iron_future_flow_k = 1.0f;
            c.iron_future_flow_decay = 0.01f;
            c.iron_future_flow_lock_th = 120.0f;
            c.iron_future_flow_congest_cap = 320.0f;
            c.iron_future_flow_zone_rows = 64;
            c.iron_future_flow_zone_cols = 64;

            // P11-only history congestion is not wired into P12's PIBT path.
            c.pibt_history_congestion_enabled = false;
            break;

        case MapType::RANDOM_DOMAIN:
            c.skip_test = false;
            // Previous random default:
            c.schedule_mode = 1;
            // c.schedule_mode = 11;   // RandomRouteLoadScheduler, w24 dynamic shortest default
            c.executor_mode = 3;    // 3
            c.planner_mode = 10;    // RandomFlowEPIBT
            c.reassign_enabled = true;
            // c.reassign_keep_bias = 0.0f;
            c.reassign_keep_bias = 3.0f;
            c.reassign_min_dist = 3;
            c.heap_dist_weight = 5.0f;
            c.game_dist_pow = 0.7f;

            // c.random_route_load_weight = 12.0f;
            // c.random_route_load_batch_size = 64;
            // c.random_route_load_decay = 0.01f;
            // c.random_route_load_length_weighted_write = true;
            // c.random_route_load_write_length_scale = 0.01f;
            // c.random_route_load_dynamic_shortest = true;
            // c.random_route_load_dynamic_path_weight = 0.1f;

            c.flow_start_turn_penalty = 2;  // RNDSTR20-I03: mild first-step straightness bias.
            c.flow_vertex_cost_mode = 4;    // Piecewise linear/quadratic tail overload cost for local congestion.
            c.flow_vertex_cost_threshold = 0;
            c.flow_vertex_cost_weight = 30;     // 40
            c.flow_vertex_cost_exp_base = 3.0f; // mode=4: quadratic tail scale.
            c.flow_vertex_cost_linear_end = 4;
            c.flow_vertex_sens_decay = 0.92f;
            c.flow_vertex_base_weight = 0;
            c.epibt_flow_vertex_mode = 4;
            c.epibt_flow_vertex_threshold = 4;
            c.epibt_flow_vertex_weight = 0;
            c.epibt_flow_vertex_cap = 1000;
            c.epibt_flow_vertex_exp_base = 0.5f; // mode=4: softer quadratic tail for execution decisions.
            c.epibt_flow_vertex_linear_end = 4;
            c.epibt_flow_vertex_base_weight = 0;

            c.heap_dist_decay_factor = 0.4;
            c.heap_dist_decay_interval_steps = 22000;
            break;

        case MapType::UNKNOWN:
            c.skip_test = true;
            break;
    }
}

// Scheduler track: 按地图保留 Scheduler 定制入口，Planner/Executor 退回官方默认。
inline void load_for_schedule(const MapProfile& profile, const SharedEnvironment* /*env*/) {
    auto& c = cfg();

    switch (profile.type) {
        case MapType::MAZE_DOMAIN:
            c.schedule_mode = 9;
            // Maze 单 Scheduler 赛道参数放这里。
            break;

        case MapType::ROOM_DOMAIN:
            c.schedule_mode = 7;
            // Room 单 Scheduler 赛道参数放这里。
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.schedule_mode = 1;
            // Warehouse 单 Scheduler 赛道参数放这里。
            break;

        case MapType::CITY_DOMAIN:
            c.schedule_mode = 1;
            // City 单 Scheduler 赛道参数放这里。
            break;

        case MapType::GAME_DOMAIN:
            c.schedule_mode = 8;
            // Game 单 Scheduler 赛道参数放这里。
            break;

        case MapType::IRON_HARVEST_DOMAIN:
            c.schedule_mode = 10;
            // Iron Harvest 单 Scheduler 赛道使用 IronZone，并固定开启 future flow。
            break;

        case MapType::RANDOM_DOMAIN:
            c.schedule_mode = 1;
            // Random 单 Scheduler 赛道参数放这里。
            break;

        case MapType::UNKNOWN:
            c.skip_test = true;
            break;
    }

    c.planner_mode = 0;
    c.executor_mode = 0;
}

// Executor track: 按地图保留 Executor 定制入口，Scheduler/Planner 退回官方默认。
inline void load_for_executor(const MapProfile& profile, const SharedEnvironment* /*env*/) {
    auto& c = cfg();

    switch (profile.type) {
        case MapType::MAZE_DOMAIN:
            c.executor_mode = 7;  // 暂无 Maze 专用 Executor，沿用 DHExecutor。
            break;

        case MapType::ROOM_DOMAIN:
            c.executor_mode = 7;  // RoomExecutor
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.executor_mode = 7;  // DHExecutor
            break;

        case MapType::CITY_DOMAIN:
            c.executor_mode = 7;  // DHExecutor
            break;

        case MapType::GAME_DOMAIN:
            c.executor_mode = 7;  // GameExecutor
            break;

        case MapType::IRON_HARVEST_DOMAIN:
            c.executor_mode = 8;  // Official executor
            break;

        case MapType::RANDOM_DOMAIN:
            c.executor_mode = 7;  // DHExecutor
            break;

        case MapType::UNKNOWN:
            c.skip_test = true;
            break;
    }

    c.schedule_mode = 0;
    c.planner_mode = 0;
}

// 根据地图类型自动调整 RuntimeConfig
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
