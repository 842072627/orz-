// optsvr/RuntimeConfig.h — 运行时参数（编译期默认值 + MapTuner + 环境变量最终覆盖）
#pragma once

#include <cstdlib>
#include <string>
#include <iostream>
#include "Config.h"

namespace optsvr {

struct RuntimeConfig {
    // Executor 执行模式枚举（Executor.cpp 直接使用）
    enum class ExecProcessNewPlanMode { ORIGIN, HYBRID, DEFAULT_EXECUTOR };
    static constexpr ExecProcessNewPlanMode EXEC_MODE = ExecProcessNewPlanMode::DEFAULT_EXECUTOR;

    // ---- 全局 ----
    bool  skip_test;
    int   debug_level;
    int   num_threads;
    bool  timing_trace;

    // ---- 模块选择 ----
    int   schedule_mode;   // 0=Default, 1=GreedyHeap, 2=DHSchedule, 3=GameOpt, 4=TaskMatch, 5=DHFULLSchedule, 6=OptMazeSchedule, 7=OptRoom, 8=GameBalanceOpt, 9=MazeOptScheduler, 10=IronZone, 11=RandomRouteLoad
    int   executor_mode;   // 0=Default, 1=Origin, 2=Hybrid, 3=DHExecutor, 4=RoomOptExecutor, 5=RoomExecutor, 6=GameExecutor, 7=IncrementalTpg, 8=ConservativeIncrementalTpg
    int   planner_mode;    // 0=Default, 1=FlowDecay, 2=GameOpt, 3=TurnAware, 4=EPIBT, 5=IdleYield, 6=RoomPlanner, 7=RoomTidalPlanner, 8=MazeTidalPlanner, 10=RandomFlowEPIBT, 11=TestUsefulFlowDecay, 12=IronFlowDecay

    // ---- GreedyHeap/TaskMatch 共用 ----
    int   score_mode;
    float heap_dist_weight;
    int   heap_rebuild_pct;
    int   heap_lns_pct;
    int   heap_sort_k;
    float heap_max_assign;
    int   heap_dist_decay_interval_steps;
    float heap_dist_decay_factor;
    int   candidate_topk;
    bool  reassign_enabled;
    float reassign_keep_bias;
    int   reassign_min_dist;
    float congestion_decay;
    float difficulty_weight;
    int   grid_cell_size;
    int   grid_radius;
    bool  congestion_use_history;

    // ---- RandomRouteLoad 专属（SCHEDULE_MODE=11）----
    float random_route_load_weight;
    int   random_route_load_batch_size;
    int   random_route_load_grid_cell;
    int   random_route_load_tight_slack;
    int   random_route_load_loose_slack;
    float random_route_load_decay;
    bool  random_route_load_length_weighted_write;
    float random_route_load_write_length_scale;
    bool  random_route_load_dynamic_shortest;
    float random_route_load_dynamic_path_weight;

    // ---- GameOpt 专属（SCHEDULE_MODE=3）----
    float game_dist_weight;
    int   game_rebuild_pct;
    int   game_lns_pct;
    int   game_sort_k;
    bool  game_reassign;
    float game_keep_bias;
    int   game_min_dist;
    float game_cong_decay;
    int   game_grid_cell;
    int   game_grid_radius;
    float game_cong_weight;
    float game_dist_pow;
    float game_dest_cong_weight;
    float game_dest_cong_cap;
    float game_age_weight;
    float game_comp_weight;
    int   game_total_steps;
    float game_assign_ratio;
    int   game_idle_dist;
    // 新版中心线调度器参数
    int   game_zone_hard_cap;
    int   game_exempt_main;
    int   game_density_cap;
    int   game_dir_threshold;
    int   game_route_penalty;
    int   game_ex_main_cap;
    int   game_ex_nonmain_cap;
    float game_bal_assign_ratio;
    // 已验证有效: 宽度感知 + 逆流惩罚 + 紧迫度
    bool  game_width_aware;
    int   game_narrow_threshold;
    float game_narrow_cap_ratio;
    bool  game_pibt_counter_flow;
    int   game_counter_flow_cost;
    bool  game_urgency_priority;
    float game_urgency_weight;
    int   game_urgency_threshold;
    // ---- 有效自适应策略 ----
    int   game_search_radius;
    int   game_late_search_radius;
    float game_late_phase_ratio;
    bool  game_late_relax;
    int   game_late_density_cap;
    int   game_late_zone_cap;
    bool  game_age_sort;
    float game_age_sort_weight;

    // ---- IronZone 专属（SCHEDULE_MODE=10）----
    int   iron_zone_rows;
    int   iron_zone_cols;
    int   iron_max_radius;
    float iron_dist_weight;
    float iron_task_len_weight;
    int   iron_long_task_th;
    int   iron_stats_interval;

    // ---- Iron future flow ----
    float iron_future_flow_w;
    float iron_future_flow_k;
    float iron_future_flow_decay;
    float iron_future_flow_lock_th;
    float iron_future_flow_congest_cap;
    int   iron_future_flow_zone_rows;
    int   iron_future_flow_zone_cols;
    float iron_future_flow_idle_weight;
    bool  iron_future_flow_highway_exempt;
    bool  iron_task_steal_enabled;
    int   iron_task_steal_dist_ratio;

    // ---- Iron lane traffic-light planner branch ----
    int   iron_lane_enabled;
    int   iron_lane_guard_dist;
    int   iron_lane_core_flip_th;
    int   iron_lane_flow_ratio_flip;
    int   iron_lane_vote_period;
    float iron_lane_idle_bonus;
    int   iron_lane_debug;
    int   iron_lane_lru_sort;
    int   iron_lane_outside_vote;
    int   iron_lane_exit_buffer_radius;
    float iron_lane_exit_priority;
    float iron_lane_buffer_penalty;
    int   iron_highway_row_penalty;
    int   iron_highway_col_penalty;
    int   iron_highway_non_highway_penalty;

    // ---- Room 部分分配 ----
    int   room_assign_agent_mode;   // 0=全部, 1=动态筛选, 2=静态随机
    float room_assign_ratio;        // 0.0~1.0
    int   room_topk_tasks;          // mode=1 采样K个task

    // ---- OptRoom 专属（SCHEDULE_MODE=7）----
    float opt_room_dist_first_w;
    float opt_room_dist_overall;
    float opt_room_short_task_w;
    int   opt_room_total_ticks;
    int   opt_room_reclaim_margin;
    float opt_room_assign_ratio;
    int   opt_room_match_mode;
    float opt_room_time_safety;
    int   opt_room_reassign;
    int   opt_room_reassign_min_dist;

    // ---- FlowDecay ----
    float flow_decay;
    int   flow_decay_scale;
    bool  pibt_forward_break_tie;
    int   guide_flow_opposite_w;
    int   guide_flow_same_w;
    bool  ware_direct;
    bool  dummy_goal_home;
    int   flow_turn_penalty;
    int   flow_start_turn_penalty;
    float flow_turn_tie;
    int   flow_vertex_cost_mode;
    int   flow_vertex_cost_threshold;
    int   flow_vertex_cost_weight;
    int   flow_vertex_cost_cap;
    float flow_vertex_cost_exp_base;
    int   flow_vertex_cost_linear_end;
    float flow_vertex_sens_decay;
    int   flow_vertex_base_weight;
    bool  p1_flow_vertex_penalty_enabled;
    bool  flow_dynamic_time_enabled;
    int   flow_dynamic_time_margin_ms;
    bool  dist2path_prewarm_enabled;
    int   dist2path_prewarm_radius;
    int   dist2path_prewarm_threads;
    int   dist2path_prewarm_min_left_ms;
    bool  pibt_history_congestion_enabled;
    int   pibt_history_window;
    float pibt_history_free_threshold;
    float pibt_history_quad_threshold;
    float pibt_history_linear_weight;
    float pibt_history_quad_weight;
    float pibt_history_cap;
    float pibt_history_quad_power;
    int   pibt_history_sort_mode;
    float pibt_history_tie_base_eps;
    bool  pibt_history_apply_to_wait;
    int   pibt_history_goal_exempt_dist;
    bool  pibt_history_stats_enabled;
    int   pibt_history_stats_interval;
    int   epibt_flow_vertex_mode;
    int   epibt_flow_vertex_threshold;
    float epibt_flow_vertex_weight;
    int   epibt_flow_vertex_cap;
    float epibt_flow_vertex_exp_base;
    int   epibt_flow_vertex_linear_end;
    int   epibt_flow_vertex_base_weight;
    bool  epibt_parallel_guide;

    // ---- RoomPlanner / RoomExecutor 优化 (精简后仅保留有效参数) ----
    int   opt_room_flow_scale;              // flow_scale覆盖
    int   opt_room_late_force_replan;       // 后期强制replan阈值
    int   opt_room_replan_start_tick;       // replan生效起始tick

    // ---- RoomTidalPlanner (Mode 7) 可调参数 ----
    int   tidal_period;                     // 潮汐投票周期
    int   tidal_guide_bonus;                // 跟随引导路径奖励
    int   tidal_backtrack_penalty;          // 回退惩罚
    int   tidal_backtrack_step;             // 回退每步惩罚
    int   tidal_wait_penalty;               // 原地等待惩罚
    int   tidal_forward_skip;               // 跳步前进惩罚
    int   tidal_off_path;                   // 偏离路径惩罚

    // ---- 距离矩阵 ----
    bool  precompute_heuristic;
    int   max_precompute_cells;
    bool  turn_aware_heuristic;

    // ---- Executor ----
    int   origin_append_steps;
    bool  congestion_adaptive_append;

    // ============ 单例 ============
    static RuntimeConfig& instance() {
        static RuntimeConfig cfg;
        return cfg;
    }

    // MapTuner 之后的最终覆盖：环境变量存在才覆盖当前值。
    void override_from_env() {
#define OVERRIDE_BOOL(env_name, field_name) \
        do { if (std::getenv(env_name)) field_name = read_bool(env_name, field_name); } while (false)
#define OVERRIDE_INT(env_name, field_name) \
        do { if (std::getenv(env_name)) field_name = read_int(env_name, field_name); } while (false)
#define OVERRIDE_FLOAT(env_name, field_name) \
        do { if (std::getenv(env_name)) field_name = read_float(env_name, field_name); } while (false)

        OVERRIDE_BOOL("OPT_SKIP_TEST", skip_test);
        OVERRIDE_INT("OPT_DEBUG_LEVEL", debug_level);
        OVERRIDE_INT("OPT_NUM_THREADS", num_threads);
        OVERRIDE_BOOL("OPT_TIMING_TRACE", timing_trace);
        OVERRIDE_INT("OPT_SCHEDULE_MODE", schedule_mode);
        OVERRIDE_INT("OPT_EXECUTOR_MODE", executor_mode);
        OVERRIDE_INT("OPT_PLANNER_MODE", planner_mode);

        OVERRIDE_INT("OPT_SCORE_MODE", score_mode);
        OVERRIDE_FLOAT("OPT_HEAP_DIST_WEIGHT", heap_dist_weight);
        OVERRIDE_INT("OPT_HEAP_REBUILD_PCT", heap_rebuild_pct);
        OVERRIDE_INT("OPT_HEAP_LNS_PCT", heap_lns_pct);
        OVERRIDE_INT("OPT_HEAP_SORT_K", heap_sort_k);
        OVERRIDE_FLOAT("OPT_HEAP_MAX_ASSIGN", heap_max_assign);
        OVERRIDE_INT("OPT_HEAP_DIST_DECAY_INTERVAL_STEPS", heap_dist_decay_interval_steps);
        OVERRIDE_FLOAT("OPT_HEAP_DIST_DECAY_FACTOR", heap_dist_decay_factor);
        OVERRIDE_INT("OPT_CANDIDATE_TOPK", candidate_topk);
        OVERRIDE_BOOL("OPT_REASSIGN_ENABLED", reassign_enabled);
        OVERRIDE_FLOAT("OPT_REASSIGN_KEEP_BIAS", reassign_keep_bias);
        OVERRIDE_INT("OPT_REASSIGN_MIN_DIST", reassign_min_dist);
        OVERRIDE_FLOAT("OPT_CONGESTION_DECAY", congestion_decay);
        OVERRIDE_FLOAT("OPT_DIFFICULTY_WEIGHT", difficulty_weight);
        OVERRIDE_INT("OPT_GRID_CELL_SIZE", grid_cell_size);
        OVERRIDE_INT("OPT_GRID_RADIUS", grid_radius);
        OVERRIDE_BOOL("OPT_CONGESTION_USE_HISTORY", congestion_use_history);
        OVERRIDE_FLOAT("OPT_RANDOM_ROUTE_LOAD_WEIGHT", random_route_load_weight);
        OVERRIDE_INT("OPT_RANDOM_ROUTE_LOAD_BATCH_SIZE", random_route_load_batch_size);
        OVERRIDE_INT("OPT_RANDOM_ROUTE_LOAD_GRID_CELL", random_route_load_grid_cell);
        OVERRIDE_INT("OPT_RANDOM_ROUTE_LOAD_TIGHT_SLACK", random_route_load_tight_slack);
        OVERRIDE_INT("OPT_RANDOM_ROUTE_LOAD_LOOSE_SLACK", random_route_load_loose_slack);
        OVERRIDE_FLOAT("OPT_RANDOM_ROUTE_LOAD_DECAY", random_route_load_decay);
        OVERRIDE_BOOL("OPT_RANDOM_ROUTE_LOAD_LENGTH_WEIGHTED_WRITE", random_route_load_length_weighted_write);
        OVERRIDE_FLOAT("OPT_RANDOM_ROUTE_LOAD_WRITE_LENGTH_SCALE", random_route_load_write_length_scale);
        OVERRIDE_BOOL("OPT_RANDOM_ROUTE_LOAD_DYNAMIC_SHORTEST", random_route_load_dynamic_shortest);
        OVERRIDE_FLOAT("OPT_RANDOM_ROUTE_LOAD_DYNAMIC_PATH_WEIGHT", random_route_load_dynamic_path_weight);

        OVERRIDE_FLOAT("GAME_DIST_WEIGHT", game_dist_weight);
        OVERRIDE_INT("GAME_REBUILD_PCT", game_rebuild_pct);
        OVERRIDE_INT("GAME_LNS_PCT", game_lns_pct);
        OVERRIDE_INT("GAME_SORT_K", game_sort_k);
        OVERRIDE_BOOL("GAME_REASSIGN", game_reassign);
        OVERRIDE_FLOAT("GAME_KEEP_BIAS", game_keep_bias);
        OVERRIDE_INT("GAME_MIN_DIST", game_min_dist);
        OVERRIDE_FLOAT("GAME_CONGESTION_DECAY", game_cong_decay);
        OVERRIDE_INT("GAME_GRID_CELL", game_grid_cell);
        OVERRIDE_INT("GAME_GRID_RADIUS", game_grid_radius);
        OVERRIDE_FLOAT("GAME_CONG_WEIGHT", game_cong_weight);
        OVERRIDE_FLOAT("GAME_DIST_POW", game_dist_pow);
        OVERRIDE_FLOAT("GAME_DEST_CONG_WEIGHT", game_dest_cong_weight);
        OVERRIDE_FLOAT("GAME_DEST_CONG_CAP", game_dest_cong_cap);
        OVERRIDE_FLOAT("GAME_AGE_WEIGHT", game_age_weight);
        OVERRIDE_FLOAT("GAME_COMP_WEIGHT", game_comp_weight);
        OVERRIDE_INT("GAME_TOTAL_STEPS", game_total_steps);
        OVERRIDE_FLOAT("GAME_ASSIGN_RATIO", game_assign_ratio);
        OVERRIDE_INT("GAME_IDLE_DIST", game_idle_dist);
        OVERRIDE_INT("GAME_ZONE_HARD_CAP", game_zone_hard_cap);
        OVERRIDE_INT("GAME_EXEMPT_MAIN", game_exempt_main);
        OVERRIDE_INT("GAME_DENSITY_CAP", game_density_cap);
        OVERRIDE_INT("GAME_DIR_THRESHOLD", game_dir_threshold);
        OVERRIDE_INT("GAME_ROUTE_PENALTY", game_route_penalty);
        OVERRIDE_INT("GAME_EX_MAIN_CAP", game_ex_main_cap);
        OVERRIDE_INT("GAME_EX_NONMAIN_CAP", game_ex_nonmain_cap);
        OVERRIDE_FLOAT("GAME_BAL_ASSIGN_RATIO", game_bal_assign_ratio);
        OVERRIDE_BOOL("GAME_WIDTH_AWARE", game_width_aware);
        OVERRIDE_INT("GAME_NARROW_THRESHOLD", game_narrow_threshold);
        OVERRIDE_FLOAT("GAME_NARROW_CAP_RATIO", game_narrow_cap_ratio);
        OVERRIDE_BOOL("GAME_PIBT_COUNTER_FLOW", game_pibt_counter_flow);
        OVERRIDE_INT("GAME_COUNTER_FLOW_COST", game_counter_flow_cost);
        OVERRIDE_BOOL("GAME_URGENCY_PRIORITY", game_urgency_priority);
        OVERRIDE_FLOAT("GAME_URGENCY_WEIGHT", game_urgency_weight);
        OVERRIDE_INT("GAME_URGENCY_THRESHOLD", game_urgency_threshold);
        OVERRIDE_INT("GAME_SEARCH_RADIUS", game_search_radius);
        OVERRIDE_INT("GAME_LATE_SEARCH_RADIUS", game_late_search_radius);
        OVERRIDE_FLOAT("GAME_LATE_PHASE_RATIO", game_late_phase_ratio);
        OVERRIDE_BOOL("GAME_LATE_RELAX", game_late_relax);
        OVERRIDE_INT("GAME_LATE_DENSITY_CAP", game_late_density_cap);
        OVERRIDE_INT("GAME_LATE_ZONE_CAP", game_late_zone_cap);
        OVERRIDE_BOOL("GAME_AGE_SORT", game_age_sort);
        OVERRIDE_FLOAT("GAME_AGE_SORT_WEIGHT", game_age_sort_weight);

        OVERRIDE_INT("IRON_ZONE_ROWS", iron_zone_rows);
        OVERRIDE_INT("IRON_ZONE_COLS", iron_zone_cols);
        OVERRIDE_INT("IRON_MAX_RADIUS", iron_max_radius);
        OVERRIDE_FLOAT("IRON_DIST_WEIGHT", iron_dist_weight);
        OVERRIDE_FLOAT("IRON_TASK_LEN_WEIGHT", iron_task_len_weight);
        OVERRIDE_INT("IRON_LONG_TASK_TH", iron_long_task_th);
        OVERRIDE_INT("IRON_STATS_INTERVAL", iron_stats_interval);
        OVERRIDE_FLOAT("IRON_FUTURE_FLOW_W", iron_future_flow_w);
        OVERRIDE_FLOAT("IRON_FUTURE_FLOW_K", iron_future_flow_k);
        OVERRIDE_FLOAT("IRON_FUTURE_FLOW_DECAY", iron_future_flow_decay);
        OVERRIDE_FLOAT("IRON_FUTURE_FLOW_LOCK_TH", iron_future_flow_lock_th);
        OVERRIDE_FLOAT("IRON_FUTURE_FLOW_CONGEST_CAP", iron_future_flow_congest_cap);
        OVERRIDE_INT("IRON_FUTURE_FLOW_ZONE_ROWS", iron_future_flow_zone_rows);
        OVERRIDE_INT("IRON_FUTURE_FLOW_ZONE_COLS", iron_future_flow_zone_cols);
        OVERRIDE_INT("IRON_LANE_ENABLED", iron_lane_enabled);
        OVERRIDE_INT("IRON_LANE_GUARD_DIST", iron_lane_guard_dist);
        OVERRIDE_INT("IRON_LANE_CORE_FLIP_TH", iron_lane_core_flip_th);
        OVERRIDE_INT("IRON_LANE_FLOW_RATIO_FLIP", iron_lane_flow_ratio_flip);
        OVERRIDE_INT("IRON_LANE_VOTE_PERIOD", iron_lane_vote_period);
        OVERRIDE_FLOAT("IRON_LANE_IDLE_BONUS", iron_lane_idle_bonus);
        OVERRIDE_INT("IRON_LANE_DEBUG", iron_lane_debug);
        OVERRIDE_INT("IRON_LANE_LRU_SORT", iron_lane_lru_sort);
        OVERRIDE_INT("IRON_LANE_OUTSIDE_VOTE", iron_lane_outside_vote);
        OVERRIDE_INT("IRON_LANE_EXIT_BUFFER_RADIUS", iron_lane_exit_buffer_radius);
        OVERRIDE_FLOAT("IRON_LANE_EXIT_PRIORITY", iron_lane_exit_priority);
        OVERRIDE_FLOAT("IRON_LANE_BUFFER_PENALTY", iron_lane_buffer_penalty);
        OVERRIDE_INT("IRON_HIGHWAY_COL_PENALTY", iron_highway_col_penalty);
        OVERRIDE_INT("IRON_HIGHWAY_NON_HIGHWAY_PENALTY", iron_highway_non_highway_penalty);

        OVERRIDE_FLOAT("OPT_FLOW_DECAY", flow_decay);
        OVERRIDE_INT("OPT_FLOW_DECAY_SCALE", flow_decay_scale);
        OVERRIDE_BOOL("OPT_PIBT_FORWARD_BREAK_TIE", pibt_forward_break_tie);
        OVERRIDE_INT("GUIDE_FLOW_OPPOSITE_W", guide_flow_opposite_w);
        OVERRIDE_INT("GUIDE_FLOW_SAME_W", guide_flow_same_w);
        OVERRIDE_BOOL("OPT_WARE_DIRECT", ware_direct);
        OVERRIDE_INT("OPT_FLOW_TURN_PENALTY", flow_turn_penalty);
        OVERRIDE_INT("OPT_FLOW_START_TURN_PENALTY", flow_start_turn_penalty);
        OVERRIDE_FLOAT("OPT_FLOW_TURN_TIE", flow_turn_tie);
        OVERRIDE_INT("OPT_FLOW_VERTEX_COST_MODE", flow_vertex_cost_mode);
        OVERRIDE_INT("OPT_FLOW_VERTEX_COST_THRESHOLD", flow_vertex_cost_threshold);
        OVERRIDE_INT("OPT_FLOW_VERTEX_COST_WEIGHT", flow_vertex_cost_weight);
        OVERRIDE_INT("OPT_FLOW_VERTEX_COST_CAP", flow_vertex_cost_cap);
        OVERRIDE_FLOAT("OPT_FLOW_VERTEX_COST_EXP_BASE", flow_vertex_cost_exp_base);
        OVERRIDE_INT("OPT_FLOW_VERTEX_COST_LINEAR_END", flow_vertex_cost_linear_end);
        OVERRIDE_FLOAT("OPT_FLOW_VERTEX_SENS_DECAY", flow_vertex_sens_decay);
        OVERRIDE_INT("OPT_FLOW_VERTEX_BASE_WEIGHT", flow_vertex_base_weight);
        OVERRIDE_BOOL("OPT_P1_FLOW_VERTEX_PENALTY_ENABLED", p1_flow_vertex_penalty_enabled);
        OVERRIDE_BOOL("OPT_FLOW_DYNAMIC_TIME_ENABLED", flow_dynamic_time_enabled);
        OVERRIDE_INT("OPT_FLOW_DYNAMIC_TIME_MARGIN_MS", flow_dynamic_time_margin_ms);
        OVERRIDE_BOOL("DIST2PATH_PREWARM_ENABLED", dist2path_prewarm_enabled);
        OVERRIDE_BOOL("OPT_DIST2PATH_PREWARM_ENABLED", dist2path_prewarm_enabled);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_RADIUS", dist2path_prewarm_radius);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_THREADS", dist2path_prewarm_threads);
        OVERRIDE_INT("OPT_DIST2PATH_PREWARM_MIN_LEFT_MS", dist2path_prewarm_min_left_ms);
        OVERRIDE_BOOL("OPT_PIBT_HISTORY_CONGESTION_ENABLED", pibt_history_congestion_enabled);
        OVERRIDE_INT("OPT_PIBT_HISTORY_WINDOW", pibt_history_window);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_FREE_THRESHOLD", pibt_history_free_threshold);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_QUAD_THRESHOLD", pibt_history_quad_threshold);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_LINEAR_WEIGHT", pibt_history_linear_weight);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_QUAD_WEIGHT", pibt_history_quad_weight);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_CAP", pibt_history_cap);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_QUAD_POWER", pibt_history_quad_power);
        OVERRIDE_INT("OPT_PIBT_HISTORY_SORT_MODE", pibt_history_sort_mode);
        OVERRIDE_FLOAT("OPT_PIBT_HISTORY_TIE_BASE_EPS", pibt_history_tie_base_eps);
        OVERRIDE_BOOL("OPT_PIBT_HISTORY_APPLY_TO_WAIT", pibt_history_apply_to_wait);
        OVERRIDE_INT("OPT_PIBT_HISTORY_GOAL_EXEMPT_DIST", pibt_history_goal_exempt_dist);
        OVERRIDE_BOOL("OPT_PIBT_HISTORY_STATS_ENABLED", pibt_history_stats_enabled);
        OVERRIDE_INT("OPT_PIBT_HISTORY_STATS_INTERVAL", pibt_history_stats_interval);
        OVERRIDE_INT("OPT_EPIBT_FLOW_VERTEX_MODE", epibt_flow_vertex_mode);
        OVERRIDE_INT("OPT_EPIBT_FLOW_VERTEX_THRESHOLD", epibt_flow_vertex_threshold);
        OVERRIDE_FLOAT("OPT_EPIBT_FLOW_VERTEX_WEIGHT", epibt_flow_vertex_weight);
        OVERRIDE_INT("OPT_EPIBT_FLOW_VERTEX_CAP", epibt_flow_vertex_cap);
        OVERRIDE_FLOAT("OPT_EPIBT_FLOW_VERTEX_EXP_BASE", epibt_flow_vertex_exp_base);
        OVERRIDE_INT("OPT_EPIBT_FLOW_VERTEX_LINEAR_END", epibt_flow_vertex_linear_end);
        OVERRIDE_INT("OPT_EPIBT_FLOW_VERTEX_BASE_WEIGHT", epibt_flow_vertex_base_weight);
        OVERRIDE_BOOL("OPT_EPIBT_PARALLEL_GUIDE", epibt_parallel_guide);

        OVERRIDE_INT("ROOM_ASSIGN_AGENT_MODE", room_assign_agent_mode);
        OVERRIDE_FLOAT("ROOM_ASSIGN_RATIO", room_assign_ratio);
        OVERRIDE_INT("ROOM_TOPK_TASKS", room_topk_tasks);

        OVERRIDE_FLOAT("OPT_ROOM_DIST_FIRST_W", opt_room_dist_first_w);
        OVERRIDE_FLOAT("OPT_ROOM_DIST_OVERALL", opt_room_dist_overall);
        OVERRIDE_FLOAT("OPT_ROOM_SHORT_TASK_W", opt_room_short_task_w);
        OVERRIDE_INT("OPT_ROOM_TOTAL_TICKS", opt_room_total_ticks);
        OVERRIDE_INT("OPT_ROOM_RECLAIM_MARGIN", opt_room_reclaim_margin);
        OVERRIDE_FLOAT("OPT_ROOM_ASSIGN_RATIO", opt_room_assign_ratio);
        OVERRIDE_INT("OPT_ROOM_MATCH_MODE", opt_room_match_mode);
        OVERRIDE_FLOAT("OPT_ROOM_TIME_SAFETY", opt_room_time_safety);
        OVERRIDE_INT("OPT_ROOM_REASSIGN", opt_room_reassign);
        OVERRIDE_INT("OPT_ROOM_REASSIGN_MIN_DIST", opt_room_reassign_min_dist);
        OVERRIDE_INT("OPT_ROOM_FLOW_SCALE", opt_room_flow_scale);
        OVERRIDE_INT("OPT_ROOM_LATE_FORCE_REPLAN", opt_room_late_force_replan);
        OVERRIDE_INT("OPT_ROOM_REPLAN_START_TICK", opt_room_replan_start_tick);

        OVERRIDE_INT("OPT_TIDAL_PERIOD", tidal_period);
        OVERRIDE_INT("OPT_TIDAL_GUIDE_BONUS", tidal_guide_bonus);
        OVERRIDE_INT("OPT_TIDAL_BACKTRACK_PENALTY", tidal_backtrack_penalty);
        OVERRIDE_INT("OPT_TIDAL_BACKTRACK_STEP", tidal_backtrack_step);
        OVERRIDE_INT("OPT_TIDAL_WAIT_PENALTY", tidal_wait_penalty);
        OVERRIDE_INT("OPT_TIDAL_FORWARD_SKIP", tidal_forward_skip);
        OVERRIDE_INT("OPT_TIDAL_OFF_PATH", tidal_off_path);

        OVERRIDE_BOOL("OPT_PRECOMPUTE_HEURISTIC", precompute_heuristic);
        OVERRIDE_INT("OPT_MAX_PRECOMPUTE_CELLS", max_precompute_cells);
        OVERRIDE_BOOL("OPT_TURN_AWARE_HEURISTIC", turn_aware_heuristic);

        OVERRIDE_INT("OPT_ORIGIN_APPEND_STEPS", origin_append_steps);
        OVERRIDE_BOOL("OPT_CONGESTION_ADAPTIVE_APPEND", congestion_adaptive_append);

#undef OVERRIDE_BOOL
#undef OVERRIDE_INT
#undef OVERRIDE_FLOAT

        if (debug_level >= 1) {
            std::cerr << "[optsvr] schedule_mode=" << schedule_mode
                      << " planner_mode=" << planner_mode << "\n";
        }
    }

private:
    RuntimeConfig() {
        skip_test = SKIP_TEST; debug_level = DEBUG_LEVEL; num_threads = NUM_THREADS;
        timing_trace = OPT_TIMING_TRACE;
        schedule_mode = SCHEDULE_MODE; executor_mode = EXECUTOR_MODE; planner_mode = PLANNER_MODE;
        score_mode = SCORE_MODE;
        heap_dist_weight = HEAP_DIST_WEIGHT; heap_rebuild_pct = HEAP_REBUILD_PCT;
        heap_lns_pct = HEAP_LNS_PCT; heap_sort_k = HEAP_SORT_K;
        heap_max_assign = HEAP_MAX_ASSIGN;
        heap_dist_decay_interval_steps = HEAP_DIST_DECAY_INTERVAL_STEPS;
        heap_dist_decay_factor = HEAP_DIST_DECAY_FACTOR;
        candidate_topk = CANDIDATE_TOPK; reassign_enabled = REASSIGN_ENABLED;
        reassign_keep_bias = REASSIGN_KEEP_BIAS; reassign_min_dist = REASSIGN_MIN_DIST;
        congestion_decay = CONGESTION_DECAY; difficulty_weight = DIFFICULTY_WEIGHT;
        grid_cell_size = CONGESTION_GRID_CELL; grid_radius = CONGESTION_GRID_RADIUS;
        congestion_use_history = CONGESTION_USE_HISTORY;
        random_route_load_weight = RANDOM_ROUTE_LOAD_WEIGHT;
        random_route_load_batch_size = RANDOM_ROUTE_LOAD_BATCH_SIZE;
        random_route_load_grid_cell = RANDOM_ROUTE_LOAD_GRID_CELL;
        random_route_load_tight_slack = RANDOM_ROUTE_LOAD_TIGHT_SLACK;
        random_route_load_loose_slack = RANDOM_ROUTE_LOAD_LOOSE_SLACK;
        random_route_load_decay = RANDOM_ROUTE_LOAD_DECAY;
        random_route_load_length_weighted_write = RANDOM_ROUTE_LOAD_LENGTH_WEIGHTED_WRITE;
        random_route_load_write_length_scale = RANDOM_ROUTE_LOAD_WRITE_LENGTH_SCALE;
        random_route_load_dynamic_shortest = RANDOM_ROUTE_LOAD_DYNAMIC_SHORTEST;
        random_route_load_dynamic_path_weight = RANDOM_ROUTE_LOAD_DYNAMIC_PATH_WEIGHT;
        flow_decay = FLOW_DECAY; flow_decay_scale = FLOW_DECAY_SCALE;
        ware_direct = WARE_DIRECT;
        dummy_goal_home = DUMMY_GOAL_HOME;
        flow_turn_penalty = OPT_FLOW_TURN_PENALTY;
        flow_start_turn_penalty = OPT_FLOW_START_TURN_PENALTY;
        flow_turn_tie = OPT_FLOW_TURN_TIE;
        flow_vertex_cost_mode = OPT_FLOW_VERTEX_COST_MODE;
        flow_vertex_cost_threshold = OPT_FLOW_VERTEX_COST_THRESHOLD;
        flow_vertex_cost_weight = OPT_FLOW_VERTEX_COST_WEIGHT;
        flow_vertex_cost_cap = OPT_FLOW_VERTEX_COST_CAP;
        flow_vertex_cost_exp_base = OPT_FLOW_VERTEX_COST_EXP_BASE;
        flow_vertex_cost_linear_end = OPT_FLOW_VERTEX_COST_LINEAR_END;
        flow_vertex_sens_decay = OPT_FLOW_VERTEX_SENS_DECAY;
        flow_vertex_base_weight = OPT_FLOW_VERTEX_BASE_WEIGHT;
        p1_flow_vertex_penalty_enabled = OPT_P1_FLOW_VERTEX_PENALTY_ENABLED;
        flow_dynamic_time_enabled = OPT_FLOW_DYNAMIC_TIME_ENABLED;
        flow_dynamic_time_margin_ms = OPT_FLOW_DYNAMIC_TIME_MARGIN_MS;
        dist2path_prewarm_enabled = OPT_DIST2PATH_PREWARM_ENABLED;
        dist2path_prewarm_radius = OPT_DIST2PATH_PREWARM_RADIUS;
        dist2path_prewarm_threads = OPT_DIST2PATH_PREWARM_THREADS;
        dist2path_prewarm_min_left_ms = OPT_DIST2PATH_PREWARM_MIN_LEFT_MS;
        pibt_history_congestion_enabled = OPT_PIBT_HISTORY_CONGESTION_ENABLED;
        pibt_history_window = OPT_PIBT_HISTORY_WINDOW;
        pibt_history_free_threshold = OPT_PIBT_HISTORY_FREE_THRESHOLD;
        pibt_history_quad_threshold = OPT_PIBT_HISTORY_QUAD_THRESHOLD;
        pibt_history_linear_weight = OPT_PIBT_HISTORY_LINEAR_WEIGHT;
        pibt_history_quad_weight = OPT_PIBT_HISTORY_QUAD_WEIGHT;
        pibt_history_cap = OPT_PIBT_HISTORY_CAP;
        pibt_history_quad_power = OPT_PIBT_HISTORY_QUAD_POWER;
        pibt_history_sort_mode = OPT_PIBT_HISTORY_SORT_MODE;
        pibt_history_tie_base_eps = OPT_PIBT_HISTORY_TIE_BASE_EPS;
        pibt_history_apply_to_wait = OPT_PIBT_HISTORY_APPLY_TO_WAIT;
        pibt_history_goal_exempt_dist = OPT_PIBT_HISTORY_GOAL_EXEMPT_DIST;
        pibt_history_stats_enabled = OPT_PIBT_HISTORY_STATS_ENABLED;
        pibt_history_stats_interval = OPT_PIBT_HISTORY_STATS_INTERVAL;
        epibt_flow_vertex_mode = OPT_EPIBT_FLOW_VERTEX_MODE;
        epibt_flow_vertex_threshold = OPT_EPIBT_FLOW_VERTEX_THRESHOLD;
        epibt_flow_vertex_weight = OPT_EPIBT_FLOW_VERTEX_WEIGHT;
        epibt_flow_vertex_cap = OPT_EPIBT_FLOW_VERTEX_CAP;
        epibt_flow_vertex_exp_base = OPT_EPIBT_FLOW_VERTEX_EXP_BASE;
        epibt_flow_vertex_linear_end = OPT_EPIBT_FLOW_VERTEX_LINEAR_END;
        epibt_flow_vertex_base_weight = OPT_EPIBT_FLOW_VERTEX_BASE_WEIGHT;
        epibt_parallel_guide = OPT_EPIBT_PARALLEL_GUIDE;

        game_dist_weight = GAME_DIST_WEIGHT; game_rebuild_pct = GAME_REBUILD_PCT;
        game_lns_pct = GAME_LNS_PCT; game_sort_k = GAME_SORT_K;
        game_reassign = GAME_REASSIGN; game_keep_bias = GAME_KEEP_BIAS;
        game_min_dist = GAME_MIN_DIST; game_cong_decay = GAME_CONG_DECAY;
        game_grid_cell = GAME_GRID_CELL; game_grid_radius = GAME_GRID_RADIUS;
        game_cong_weight = GAME_CONG_WEIGHT; game_dist_pow = GAME_DIST_POW;
        game_dest_cong_weight = GAME_DEST_CONG_WEIGHT;
        game_dest_cong_cap = GAME_DEST_CONG_CAP;
        game_age_weight = GAME_AGE_WEIGHT;
        game_comp_weight = GAME_COMP_WEIGHT;
        game_total_steps = GAME_TOTAL_STEPS;
        game_assign_ratio = GAME_ASSIGN_RATIO;
        game_idle_dist = GAME_IDLE_DIST;
        game_zone_hard_cap = GAME_ZONE_HARD_CAP;
        game_exempt_main = GAME_EXEMPT_MAIN;
        game_density_cap = GAME_DENSITY_CAP;
        game_dir_threshold = GAME_DIR_THRESHOLD;
        game_route_penalty = GAME_ROUTE_PENALTY;
        game_ex_main_cap = GAME_EX_MAIN_CAP;
        game_ex_nonmain_cap = GAME_EX_NONMAIN_CAP;
        game_bal_assign_ratio = GAME_BAL_ASSIGN_RATIO;
        game_width_aware = GAME_WIDTH_AWARE;
        game_narrow_threshold = GAME_NARROW_THRESHOLD;
        game_narrow_cap_ratio = GAME_NARROW_CAP_RATIO;
        game_pibt_counter_flow = GAME_PIBT_COUNTER_FLOW;
        game_counter_flow_cost = GAME_COUNTER_FLOW_COST;
        game_urgency_priority = GAME_URGENCY_PRIORITY;
        game_urgency_weight = GAME_URGENCY_WEIGHT;
        game_urgency_threshold = GAME_URGENCY_THRESHOLD;
        // 有效自适应策略
        game_search_radius = GAME_SEARCH_RADIUS;
        game_late_search_radius = GAME_LATE_SEARCH_RADIUS;
        game_late_phase_ratio = GAME_LATE_PHASE_RATIO;
        game_late_relax = GAME_LATE_RELAX;
        game_late_density_cap = GAME_LATE_DENSITY_CAP;
        game_late_zone_cap = GAME_LATE_ZONE_CAP;
        game_age_sort = GAME_AGE_SORT;
        game_age_sort_weight = GAME_AGE_SORT_WEIGHT;

        iron_zone_rows = IRON_ZONE_ROWS;
        iron_zone_cols = IRON_ZONE_COLS;
        iron_max_radius = IRON_MAX_RADIUS;
        iron_dist_weight = IRON_DIST_WEIGHT;
        iron_task_len_weight = IRON_TASK_LEN_WEIGHT;
        iron_long_task_th = IRON_LONG_TASK_TH;
        iron_stats_interval = IRON_STATS_INTERVAL;
        iron_future_flow_w = IRON_FUTURE_FLOW_W;
        iron_future_flow_k = IRON_FUTURE_FLOW_K;
        iron_future_flow_decay = IRON_FUTURE_FLOW_DECAY;
        iron_future_flow_lock_th = IRON_FUTURE_FLOW_LOCK_TH;
        iron_future_flow_congest_cap = IRON_FUTURE_FLOW_CONGEST_CAP;
        iron_future_flow_zone_rows = IRON_FUTURE_FLOW_ZONE_ROWS;
        iron_future_flow_zone_cols = IRON_FUTURE_FLOW_ZONE_COLS;
        iron_lane_enabled = IRON_LANE_ENABLED;
        iron_lane_guard_dist = IRON_LANE_GUARD_DIST;
        iron_lane_core_flip_th = IRON_LANE_CORE_FLIP_TH;
        iron_lane_flow_ratio_flip = IRON_LANE_FLOW_RATIO_FLIP;
        iron_lane_vote_period = IRON_LANE_VOTE_PERIOD;
        iron_lane_idle_bonus = IRON_LANE_IDLE_BONUS;
        iron_lane_debug = IRON_LANE_DEBUG;
        iron_lane_lru_sort = IRON_LANE_LRU_SORT;
        iron_lane_outside_vote = IRON_LANE_OUTSIDE_VOTE;
        iron_lane_exit_buffer_radius = IRON_LANE_EXIT_BUFFER_RADIUS;
        iron_lane_exit_priority = IRON_LANE_EXIT_PRIORITY;
        iron_lane_buffer_penalty = IRON_LANE_BUFFER_PENALTY;
        iron_highway_col_penalty = IRON_HIGHWAY_COL_PENALTY;
        iron_highway_non_highway_penalty = IRON_HIGHWAY_NON_HIGHWAY_PENALTY;

        room_assign_agent_mode = ROOM_ASSIGN_AGENT_MODE;
        room_assign_ratio = ROOM_ASSIGN_RATIO;
        room_topk_tasks = ROOM_TOPK_TASKS;

        // OptRoom 专属
        opt_room_dist_first_w = OPT_ROOM_DIST_FIRST_W;
        opt_room_dist_overall = OPT_ROOM_DIST_OVERALL;
        opt_room_short_task_w = OPT_ROOM_SHORT_TASK_W;
        opt_room_total_ticks = OPT_ROOM_TOTAL_TICKS;
        opt_room_reclaim_margin = OPT_ROOM_RECLAIM_MARGIN;
        opt_room_assign_ratio = OPT_ROOM_ASSIGN_RATIO;
        opt_room_match_mode = OPT_ROOM_MATCH_MODE;
        opt_room_time_safety = OPT_ROOM_TIME_SAFETY;
        opt_room_reassign = OPT_ROOM_REASSIGN;
        opt_room_reassign_min_dist = OPT_ROOM_REASSIGN_MIN_DIST;

        // RoomPlanner / RoomExecutor 优化 (精简后仅保留有效参数)
        opt_room_flow_scale = OPT_ROOM_FLOW_SCALE;
        opt_room_late_force_replan = OPT_ROOM_LATE_FORCE_REPLAN;
        opt_room_replan_start_tick = OPT_ROOM_REPLAN_START_TICK;

        // RoomTidalPlanner (Mode 7) 可调参数
        tidal_period = OPT_TIDAL_PERIOD;
        tidal_guide_bonus = OPT_TIDAL_GUIDE_BONUS;
        tidal_backtrack_penalty = OPT_TIDAL_BACKTRACK_PENALTY;
        tidal_backtrack_step = OPT_TIDAL_BACKTRACK_STEP;
        tidal_wait_penalty = OPT_TIDAL_WAIT_PENALTY;
        tidal_forward_skip = OPT_TIDAL_FORWARD_SKIP;
        tidal_off_path = OPT_TIDAL_OFF_PATH;

        precompute_heuristic = PRECOMPUTE_HEURISTIC;
        max_precompute_cells = MAX_PRECOMPUTE_CELLS;
        turn_aware_heuristic = TURN_AWARE_HEURISTIC;
        origin_append_steps = ORIGIN_APPEND_STEPS;
        congestion_adaptive_append = CONGESTION_ADAPTIVE_APPEND;

        pibt_forward_break_tie = OPT_PIBT_FORWARD_BREAK_TIE;
        guide_flow_opposite_w = GUIDE_FLOW_OPPOSITE_W;
        guide_flow_same_w = GUIDE_FLOW_SAME_W;
    }

    static bool read_bool(const char* n, bool d) {
        const char* v = std::getenv(n); if (!v) return d;
        std::string s(v); return s=="1"||s=="true"||s=="True"||s=="TRUE";
    }
    static int read_int(const char* n, int d) {
        const char* v = std::getenv(n); if (!v) return d;
        try { return std::stoi(v); } catch(...) { return d; }
    }
    static float read_float(const char* n, float d) {
        const char* v = std::getenv(n); if (!v) return d;
        try { return std::stof(v); } catch(...) { return d; }
    }
};

inline RuntimeConfig& cfg() { return RuntimeConfig::instance(); }

} // namespace optsvr
