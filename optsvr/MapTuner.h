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
            c.skip_test = true;
            c.schedule_mode = 9;    // 9
            c.planner_mode = 8;
            c.executor_mode = 3;    // 3

            c.dh_executor_ceil_window = true; // RoomExecutor is DH-derived; use ceil window sizing

            // MazeOptScheduler mode=9 mirrors the ROOM stable-snatch shape:
            // full assignment early, then partial min-cost-flow after switch_tick.
            c.opt_maze_switch_tick = 3500;
            c.opt_maze_assign_ratio = 0.80f;
            c.opt_maze_snatch_min_pickup_distance = 5;
            c.opt_maze_cell_size = 2;
            c.opt_maze_dist_first_w = 4.0f;
            c.opt_maze_late_dist_first_w = c.opt_maze_dist_first_w;
            c.opt_maze_dist_overall = 0.9f;
            c.opt_maze_density_start_w = -0.1f;
            c.opt_maze_density_end_w = 0.2f;

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

            c.dh_executor_ceil_window = true; // RoomExecutor is DH-derived; use ceil window sizing

            // 最优参数 (R47-IDEA6 验证: avg=742.3, peak=756)
            c.opt_room_assign_ratio   = 0.77f;  // R44~R49验证: 0.75稳定最优
            c.opt_room_time_safety    = 1.2f;   // legacy match_mode 0/1/2 safety factor
            c.opt_room_match_mode     = 3;      // IDEA/room-0628: min-cost-flow top80 stable-snatch
            c.opt_room_dist_first_w   = 2.0f;
            c.opt_room_snatch_min_pickup_distance = 5;
            c.opt_room_snatch_min_abs_improve = 3.0f;
            c.opt_room_snatch_min_rel_improve = 0.00f;
            c.opt_room_idle_mode = 0;  // no-task agents stay at current rollout cell with very low priority
            // RoomTidalPlanner 对齐 IDEA/room0705 默认链路；任务分配调优保持上方 R47 参数。
            c.tidal_period            = 80;
            c.tidal_off_path          = 45;
            c.tidal_backtrack_penalty = 45;
            c.tidal_backtrack_step    = 8;
            c.tidal_guide_bonus       = 30;
            c.tidal_wait_penalty      = 8;
            c.tidal_forward_skip      = 4;
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = false;
            if (profile.n_agents > 7000) {
                // Large warehouse maps use the migrated IDEA/warehouse_new line.
                c.schedule_mode = 13;  // HardWarehouseScheduler
                c.planner_mode = 13;   // HardWarehouseFlowDecay
                c.executor_mode = 3;   // DHExecutor

                c.reassign_enabled = true;
                c.reassign_keep_bias = 4.0f;
                c.reassign_min_dist = 7;

                c.precompute_heuristic = true;
                c.ware_direct = true;
                c.ware_idle_spawn_goal = true;
                c.ware_idle_low_load_parking = false;
                c.pibt_history_congestion_enabled = false;

                c.hard_ware_direct = true;
                c.hard_dummy_goal_home = true;
                c.hard_fullfill_highway_force_move = false;
                c.hard_heuristic_hard_direction = true;

                c.hard_zone_rows = 19;
                c.hard_zone_cols = 18;
                c.hard_max_radius = 3;
                c.hard_dist_weight = 6.0f;
                c.hard_task_len_weight = 1.25f;
                c.hard_long_task_th = 1250;
                c.hard_stats_interval = 10;

                c.hard_future_flow_w = 1.0f;
                c.hard_future_flow_k = 1.0f;
                c.hard_future_flow_decay = 0.01f;
                c.hard_future_flow_lock_th = 0.0f;
                c.hard_future_flow_congest_cap = 0.0f;
                c.hard_future_flow_zone_rows = 40;
                c.hard_future_flow_zone_cols = 20;
                c.hard_ff_zone_tuning_enabled = true;
                c.hard_ff_edge_cols = 1;
                c.hard_ff_topbot_rows = 1;
                c.hard_ff_left_edge_congest_cap = 75.0f;
                c.hard_ff_right_edge_congest_cap = 200.0f;
                c.hard_ff_top_congest_cap = 200.0f;
                c.hard_ff_bottom_congest_cap = 200.0f;
                c.hard_ff_left_corner_penalty = 0.0f;
                c.hard_ff_right_corner_penalty = 0.0f;
                c.hard_ff_outer_penalty = 0.0f;
                c.hard_ff_left_edge_penalty = 0.0f;
                c.hard_ff_idle_w_dist = 2.5f;
                c.hard_ff_idle_w_repel = 10.0f;
                c.hard_ff_flow_block_pct = 0.95f;
                c.hard_ff_flow_lock_pct = 0.0f;
            } else {
                c.skip_test = true;
                // Smaller warehouse maps keep the previous MAIN2026 MapTuner default.
                c.schedule_mode = 1;
                c.planner_mode = 11;  // test_useful FlowDecay copy
                c.executor_mode = 3;  // DHExecutor

                c.ware_direct = true;
                //c.flow_continue_next_errand = true;

                //c.heap_dist_decay_interval_steps = 22000;
                //c.heap_dist_decay_factor = 0.5f;
                //c.heap_dist_decay_max_assign = 0.85f;

                //c.ware_idle_spawn_goal = true;
            }
            break;

        case MapType::CITY_DOMAIN:
            c.skip_test = true;
            c.planner_mode = 1;
            c.schedule_mode = 1;
            c.executor_mode = 7;

            c.flow_vertex_cost_weight = 40;     // 40

            //c.flow_guide_cost_to_path_weight = 2.0f;

            c.flow_continue_next_errand = true;
            c.heap_dist_decay_factor = 0.3;
            c.heap_dist_decay_interval_steps = 7000;
            c.heap_dist_decay_max_assign = 0.85f;
            break;

        case MapType::GAME_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 8;  // GameBalanceOptScheduler (中心线调度)
            c.executor_mode = 6;  // 6
            c.planner_mode  = 16; // GameDirectionalPlanner (ORZ highway directions)
            c.precompute_heuristic = true;

            c.reassign_enabled = false;
            c.game_heuristic_hard_direction = true;
            c.game_idle_highway_relocate = true;
            c.game_skip_channel_errands = false;
            c.game_dir_flow_enabled = true;
            c.game_dir_flow_cap = 140;
            c.game_dir_flow_gap = 0;
            c.game_dir_flow_decay = 0.9f;
            c.game_region_match = true;
            c.game_region_flow_batch = 64;
            // 已验证最优参数
            // c.game_width_aware       = true;   // 瓶颈宽度感知容量控制
            // c.game_narrow_threshold  = 4;      // 宽度<=4视为窄瓶颈
            // c.game_narrow_cap_ratio  = 0.5f;   // 窄瓶颈容量 = width * 0.5
            // c.game_pibt_counter_flow = true;   // PIBT逆流惩罚
            // c.game_counter_flow_cost = 20;     // 逆流额外代价
            // c.game_urgency_priority  = true;   // 紧迫度优先级
            // c.game_urgency_weight    = 10.0f;  // urgency weight
            // c.game_urgency_threshold = 50;     // urgency 阈值
            // c.game_distance_score_mode = 0;    // 默认距离打分: 1/(dist+1)
            // c.game_search_radius     = 3;      // ORZ highway mode region search radius
            // //c.game_total_steps       = 40000;
            // c.game_late_search_radius = 3;     // 后期搜索半径
            // c.game_late_search_enable = true;  // 后期扩大搜索半径
            // c.game_late_phase_ratio  = 0.4f;   // 40%后进入后期模式
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
            c.dh_executor_ceil_window = true;
            c.planner_mode = 10;    // RandomFlowEPIBT
            c.reassign_enabled = true;
            // c.reassign_keep_bias = 0.0f;
            c.reassign_keep_bias = 3.0f;
            c.reassign_min_dist = 5;
            c.heap_dist_weight = 4.0f;
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
            c.flow_vertex_cost_weight = 40;     // 40
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
            c.heap_dist_decay_max_assign = 0.85f;
            
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
            //c.opt_maze_switch_tick = 0;        // 从一开始就启用 partial assignment
            //c.opt_maze_assign_ratio = 0.7f;
            break;

        case MapType::ROOM_DOMAIN:
            c.schedule_mode = 7;
            // Room 单 Scheduler 赛道参数放这里。
            c.opt_room_assign_ratio = 0.65;
            c.opt_room_match_mode = 3;
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.schedule_mode = 13;
            c.hard_zone_rows = 19;
            c.hard_zone_cols = 18;
            c.hard_max_radius = 1;
            c.hard_dist_weight = 6.0f;
            c.hard_task_len_weight = 1.25f;
            c.hard_future_flow_lock_th = 0.0f;
            c.hard_future_flow_congest_cap = 320.0f;
            c.hard_future_flow_zone_rows = 40;
            c.hard_future_flow_zone_cols = 20;
            c.hard_ff_zone_tuning_enabled = true;
            c.hard_ff_left_edge_congest_cap = 75.0f;
            c.hard_ff_right_edge_congest_cap = 200.0f;
            c.hard_ff_top_congest_cap = 200.0f;
            c.hard_ff_bottom_congest_cap = 200.0f;
            c.hard_ff_idle_w_dist = 2.5f;
            c.hard_ff_idle_w_repel = 10.0f;
            c.hard_ff_flow_block_pct = 0.95f;
            c.hard_ff_flow_lock_pct = 0.0f;
            break;

        case MapType::CITY_DOMAIN:
            c.schedule_mode = 1;
            // City 单 Scheduler 赛道参数放这里。
            c.heap_max_assign = 0.8f;
            break;

        case MapType::GAME_DOMAIN:
            c.schedule_mode = 8;
            // Game 单 Scheduler 赛道参数放这里。
            c.game_bal_assign_ratio = 0.6f;
            break;

        case MapType::IRON_HARVEST_DOMAIN:
            c.schedule_mode = 10;
            // Iron Harvest 单 Scheduler 赛道使用 IronZone，并固定开启 future flow。
            c.iron_highway_quota_enabled = true;
            c.iron_highway_quota_limit = 800;
            break;

        case MapType::RANDOM_DOMAIN:
            c.schedule_mode = 1;
            // Random 单 Scheduler 赛道参数放这里。
            //c.heap_max_assign = 0.9f;
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
            c.executor_mode = 5;  // RoomExecutor
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.executor_mode = 3;  // DHExecutor
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
