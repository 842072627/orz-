// optsvr/RuntimeConfig.h — 运行时参数（环境变量覆盖编译期默认值）
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

    // ---- 模块选择 ----
    int   schedule_mode;   // 0=Default, 1=GreedyHeap, 2=DHSchedule, 3=GameOpt, 4=TaskMatch, 5=DHFULLSchedule, 7=OptRoom, 8=GameBalanceOpt, 9=GameSegmentFlow, 10=IronZone
    int   executor_mode;   // 0=Default, 1=Origin, 2=Hybrid, 3=DHExecutor, 4=RoomOptExecutor, 5=RoomExecutor, 6=GameExecutor
    int   planner_mode;    // 0=Default, 1=FlowDecay, 2=GameOpt, 3=TurnAware, 4=EPIBT, 5=IdleYield, 6=RoomPlanner, 7=RoomTidalPlanner, 8=GameNarrowPIBT, 9=OptMazePlanner

    // ---- GreedyHeap/TaskMatch 共用 ----
    int   score_mode;
    float heap_dist_weight;
    int   heap_rebuild_pct;
    int   heap_lns_pct;
    int   heap_sort_k;
    float heap_max_assign;
    int   candidate_topk;
    bool  reassign_enabled;
    float reassign_keep_bias;
    int   reassign_min_dist;
    float congestion_decay;
    float difficulty_weight;
    int   grid_cell_size;
    int   grid_radius;
    bool  congestion_use_history;

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
    int   game_narrow_pibt_idea;
    int   game_narrow_wait_bonus;
    int   game_narrow_density_threshold;
    int   game_narrow_base_counter_pct;
    int   game_narrow_late_start;
    int   game_narrow_late_extra;
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

    // ---- OptMaze 专属（SCHEDULE_MODE=6 / PLANNER_MODE=9）----
    int   opt_maze_cell_size;
    float opt_maze_dist_first_w;
    float opt_maze_dist_overall;
    float opt_maze_density_start;
    float opt_maze_density_end;
    int   opt_maze_reclaim_margin;
    bool  opt_maze_time_filter;
    int   opt_maze_total_ticks;
    int   opt_maze_greedy;
    int   opt_maze_lookahead;
    float opt_maze_lookahead_w;
    float opt_maze_congestion_w;
    int   opt_maze_dynamic_fw;
    float opt_maze_fw_early;
    float opt_maze_fw_late;
    int   opt_maze_lookahead_topk;
    bool  opt_maze_assign_flow;
    float opt_maze_assign_reverse_w;
    float opt_maze_assign_entry_w;
    float opt_maze_assign_lane_w;
    float opt_maze_assign_align_bonus;

    bool  opt_maze_planner_highway;
    bool  opt_maze_planner_split;
    bool  opt_maze_planner_tidal;
    bool  opt_maze_planner_guide;
    bool  opt_maze_planner_lane_aware;
    bool  opt_maze_planner_intent_priority;
    bool  opt_maze_planner_intent_entry;
    bool  opt_maze_planner_intent_reverse;
    bool  opt_maze_planner_intent_corridor_only;
    bool  opt_maze_planner_entry_arbiter;
    bool  opt_maze_planner_entry_demote;
    bool  opt_maze_planner_entry_winner;
    bool  opt_maze_planner_tidal_majority;
    int   opt_maze_planner_highway_penalty;
    int   opt_maze_planner_split_penalty;
    int   opt_maze_planner_tidal_period;
    float opt_maze_planner_tidal_delta;
    int   opt_maze_planner_lane_flow_penalty;
    int   opt_maze_planner_lane_headon_penalty;
    int   opt_maze_planner_lane_reverse_penalty;
    int   opt_maze_planner_lane_wait_penalty;
    int   opt_maze_planner_lane_refuge_bonus;
    int   opt_maze_planner_intent_entry_min;
    float opt_maze_planner_intent_delta;
    int   opt_maze_planner_entry_min;
    float opt_maze_planner_entry_delta;

    // ---- IronZone 专属（SCHEDULE_MODE=10）----
    int   iron_zone_rows;
    int   iron_zone_cols;
    int   iron_assign_variant;
    int   iron_local_radius;
    int   iron_max_radius;
    int   iron_candidate_topk;
    float iron_dist_weight;
    float iron_task_len_weight;
    float iron_cross_zone_w;
    float iron_endpoint_w;
    float iron_age_weight;
    float iron_assign_ratio;
    float iron_balance_w;
    float iron_migration_w;
    int   iron_balance_cap;
    int   iron_short_task_topk;
    int   iron_starved_task_topk;
    int   iron_starved_zone_threshold;
    int   iron_long_task_th;
    int   iron_stats_interval;
    int   iron_early_phase_ticks;
    float iron_early_dist_w;
    float iron_early_cross_w;
    float iron_early_long_w;
    bool  iron_aggressive_assign;
    int   iron_aggressive_iters;
    int   iron_aggressive_probe;
    float iron_aggressive_replace_margin;
    float iron_opportunity_ratio;
    int   iron_opportunity_dist;
    int   iron_opportunity_completion;
    int   iron_opportunity_cross;

    // Iron 红绿灯
    int   iron_lane_enabled;
    int   iron_lane_guard_dist;
    int   iron_lane_core_flip_th;
    int   iron_lane_flow_ratio_flip;
    int   iron_lane_vote_period;
    float iron_lane_idle_bonus;
    int   iron_lane_debug;
    int   iron_lane_lru_sort;

    // Iron future flow
    float iron_future_flow_weight;
    float iron_future_flow_k;
    float iron_future_flow_decay;
    float iron_future_flow_idle1;
    float iron_future_flow_idle2;
    float iron_future_flow_busy1;
    int   iron_future_flow_predict;
    float iron_future_flow_reward_w;
    float iron_future_flow_penalty_w;
    float iron_future_flow_lock_w;
    float iron_future_flow_lock_th;

    // Iron future flow zone grid (independent from scheduler's zone grid)
    int   iron_future_flow_zone_rows;
    int   iron_future_flow_zone_cols;

    // ---- FlowDecay ----
    float flow_decay;
    int   flow_decay_scale;
    bool  ware_direct;
    bool  pibt_forward_break_tie;
    int   guide_flow_opposite_w;
    int   guide_flow_same_w;

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
    int   tidal_priority_delta;             // 潮汐方向优先级差值
    int   tidal_refuge_penalty;             // 避难区入口惩罚
    int   tidal_pibt_retry_limit;           // PIBT重试次数
    // R57/R58: Executor yield策略
    int   room_exec_yield_highway;          // 通道让路: R57 avg=731.7但有deadlock风险
    int   room_exec_yield_thresh;           // yield触发阈值(连续STOP次数)
    int   room_pibt_yield;                  // PIBT中STOP agent降优先级
    int   room_pibt_yield_penalty;          // STOP agent优先级减量

    // R59: Schedule层新策略
    int   opt_room_late_fw_phase;           // 后期fw调整
    float opt_room_late_fw_value;           // 后期fw值
    int   opt_room_late_fw_start_tick;      // 后期开始tick
    int   opt_room_progress_boost;          // 任务进度agent优先
    float opt_room_progress_boost_w;        // 进度boost权重

    // R59: Planner层新策略
    int   tidal_corridor_forward;           // 通道PIBT引导前进
    int   tidal_corridor_forward_bonus;     // 通道前进奖励

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

    void load_from_env() {
        debug_level      = read_int ("OPT_DEBUG_LEVEL",      DEBUG_LEVEL);
        num_threads      = read_int ("OPT_NUM_THREADS",      NUM_THREADS);
        schedule_mode    = read_int ("OPT_SCHEDULE_MODE",    SCHEDULE_MODE);
        executor_mode    = read_int ("OPT_EXECUTOR_MODE",    EXECUTOR_MODE);
        planner_mode     = read_int ("OPT_PLANNER_MODE",     PLANNER_MODE);

        score_mode       = read_int ("OPT_SCORE_MODE",       SCORE_MODE);
        heap_dist_weight = read_float("OPT_HEAP_DIST_WEIGHT",HEAP_DIST_WEIGHT);
        heap_rebuild_pct = read_int ("OPT_HEAP_REBUILD_PCT", HEAP_REBUILD_PCT);
        heap_lns_pct     = read_int ("OPT_HEAP_LNS_PCT",     HEAP_LNS_PCT);
        heap_sort_k      = read_int ("OPT_HEAP_SORT_K",      HEAP_SORT_K);
        heap_max_assign  = read_float("OPT_HEAP_MAX_ASSIGN",  HEAP_MAX_ASSIGN);
        candidate_topk   = read_int ("OPT_CANDIDATE_TOPK",   CANDIDATE_TOPK);
        reassign_enabled = read_bool("OPT_REASSIGN_ENABLED", REASSIGN_ENABLED);
        reassign_keep_bias = read_float("OPT_REASSIGN_KEEP_BIAS", REASSIGN_KEEP_BIAS);
        reassign_min_dist  = read_int("OPT_REASSIGN_MIN_DIST", REASSIGN_MIN_DIST);
        congestion_decay   = read_float("OPT_CONGESTION_DECAY", CONGESTION_DECAY);
        difficulty_weight  = read_float("OPT_DIFFICULTY_WEIGHT", DIFFICULTY_WEIGHT);
        grid_cell_size     = read_int("OPT_GRID_CELL_SIZE",  CONGESTION_GRID_CELL);
        grid_radius        = read_int("OPT_GRID_RADIUS",     CONGESTION_GRID_RADIUS);
        congestion_use_history = read_bool("OPT_CONGESTION_USE_HISTORY", CONGESTION_USE_HISTORY);

        flow_decay       = read_float("OPT_FLOW_DECAY",      FLOW_DECAY);
        flow_decay_scale = read_int  ("OPT_FLOW_DECAY_SCALE",FLOW_DECAY_SCALE);
        ware_direct      = read_bool ("OPT_WARE_DIRECT",     WARE_DIRECT);
        pibt_forward_break_tie =
            read_bool("OPT_PIBT_FORWARD_BREAK_TIE", OPT_PIBT_FORWARD_BREAK_TIE);
        guide_flow_opposite_w = read_int("GUIDE_FLOW_OPPOSITE_W", GUIDE_FLOW_OPPOSITE_W);
        guide_flow_same_w     = read_int("GUIDE_FLOW_SAME_W",     GUIDE_FLOW_SAME_W);

        // RoomPlanner / RoomExecutor 优化 (精简后仅保留有效参数)
        opt_room_flow_scale          = read_int  ("OPT_ROOM_FLOW_SCALE",      OPT_ROOM_FLOW_SCALE);
        opt_room_late_force_replan   = read_int  ("OPT_ROOM_LATE_FORCE_REPLAN", OPT_ROOM_LATE_FORCE_REPLAN);
        opt_room_replan_start_tick   = read_int  ("OPT_ROOM_REPLAN_START_TICK", OPT_ROOM_REPLAN_START_TICK);

        // RoomTidalPlanner (Mode 7) 可调参数
        tidal_period             = read_int("OPT_TIDAL_PERIOD",           OPT_TIDAL_PERIOD);
        tidal_guide_bonus        = read_int("OPT_TIDAL_GUIDE_BONUS",      OPT_TIDAL_GUIDE_BONUS);
        tidal_backtrack_penalty  = read_int("OPT_TIDAL_BACKTRACK_PENALTY", OPT_TIDAL_BACKTRACK_PENALTY);
        tidal_backtrack_step     = read_int("OPT_TIDAL_BACKTRACK_STEP",    OPT_TIDAL_BACKTRACK_STEP);
        tidal_wait_penalty       = read_int("OPT_TIDAL_WAIT_PENALTY",      OPT_TIDAL_WAIT_PENALTY);
        tidal_forward_skip       = read_int("OPT_TIDAL_FORWARD_SKIP",      OPT_TIDAL_FORWARD_SKIP);
        tidal_off_path           = read_int("OPT_TIDAL_OFF_PATH",          OPT_TIDAL_OFF_PATH);
        tidal_priority_delta     = read_int("OPT_TIDAL_PRIORITY_DELTA",    OPT_TIDAL_PRIORITY_DELTA);
        tidal_refuge_penalty     = read_int("OPT_TIDAL_REFUGE_PENALTY",    OPT_TIDAL_REFUGE_PENALTY);
        tidal_pibt_retry_limit   = read_int("OPT_TIDAL_PIBT_RETRY_LIMIT", OPT_TIDAL_PIBT_RETRY_LIMIT);
        // Executor yield策略 (R57/R58)
        room_exec_yield_highway  = read_int("OPT_ROOM_EXEC_YIELD_HIGHWAY", OPT_ROOM_EXEC_YIELD_HIGHWAY);
        room_exec_yield_thresh   = read_int("OPT_ROOM_EXEC_YIELD_THRESH", OPT_ROOM_EXEC_YIELD_THRESH);
        room_pibt_yield          = read_int("OPT_ROOM_PIBT_YIELD", OPT_ROOM_PIBT_YIELD);
        room_pibt_yield_penalty  = read_int("OPT_ROOM_PIBT_YIELD_PENALTY", OPT_ROOM_PIBT_YIELD_PENALTY);

        // R59: Schedule层新策略
        opt_room_late_fw_phase       = read_int  ("OPT_ROOM_LATE_FW_PHASE",      OPT_ROOM_LATE_FW_PHASE);
        opt_room_late_fw_value       = read_float("OPT_ROOM_LATE_FW_VALUE",      OPT_ROOM_LATE_FW_VALUE);
        opt_room_late_fw_start_tick  = read_int  ("OPT_ROOM_LATE_FW_START_TICK", OPT_ROOM_LATE_FW_START_TICK);
        opt_room_progress_boost      = read_int  ("OPT_ROOM_PROGRESS_BOOST",     OPT_ROOM_PROGRESS_BOOST);
        opt_room_progress_boost_w    = read_float("OPT_ROOM_PROGRESS_BOOST_W",   OPT_ROOM_PROGRESS_BOOST_W);
        // R59: Planner层新策略
        tidal_corridor_forward       = read_int  ("OPT_TIDAL_CORRIDOR_FORWARD",        OPT_TIDAL_CORRIDOR_FORWARD);
        tidal_corridor_forward_bonus = read_int  ("OPT_TIDAL_CORRIDOR_FORWARD_BONUS",  OPT_TIDAL_CORRIDOR_FORWARD_BONUS);

        // Room 部分分配
        room_assign_agent_mode = read_int  ("ROOM_ASSIGN_AGENT_MODE", ROOM_ASSIGN_AGENT_MODE);
        room_assign_ratio      = read_float("ROOM_ASSIGN_RATIO",      ROOM_ASSIGN_RATIO);
        room_topk_tasks        = read_int  ("ROOM_TOPK_TASKS",        ROOM_TOPK_TASKS);

        // OptRoom 专属
        opt_room_dist_first_w    = read_float("OPT_ROOM_DIST_FIRST_W",    OPT_ROOM_DIST_FIRST_W);
        opt_room_dist_overall    = read_float("OPT_ROOM_DIST_OVERALL",    OPT_ROOM_DIST_OVERALL);
        opt_room_short_task_w    = read_float("OPT_ROOM_SHORT_TASK_W",    OPT_ROOM_SHORT_TASK_W);
        opt_room_total_ticks     = read_int  ("OPT_ROOM_TOTAL_TICKS",     OPT_ROOM_TOTAL_TICKS);
        opt_room_reclaim_margin  = read_int  ("OPT_ROOM_RECLAIM_MARGIN",  OPT_ROOM_RECLAIM_MARGIN);
        opt_room_assign_ratio    = read_float("OPT_ROOM_ASSIGN_RATIO",   OPT_ROOM_ASSIGN_RATIO);
        opt_room_match_mode      = read_int  ("OPT_ROOM_MATCH_MODE",     OPT_ROOM_MATCH_MODE);
        opt_room_time_safety     = read_float("OPT_ROOM_TIME_SAFETY",   OPT_ROOM_TIME_SAFETY);
        opt_room_reassign        = read_int  ("OPT_ROOM_REASSIGN",       OPT_ROOM_REASSIGN);
        opt_room_reassign_min_dist = read_int("OPT_ROOM_REASSIGN_MIN_DIST", OPT_ROOM_REASSIGN_MIN_DIST);

        // OptMaze 专属
        opt_maze_cell_size       = read_int  ("OPT_MAZE_CELL_SIZE",       OPT_MAZE_CELL_SIZE);
        opt_maze_dist_first_w    = read_float("OPT_MAZE_DIST_FIRST_W",    OPT_MAZE_DIST_FIRST_W);
        opt_maze_dist_overall    = read_float("OPT_MAZE_DIST_OVERALL",    OPT_MAZE_DIST_OVERALL);
        opt_maze_density_start   = read_float("OPT_MAZE_DENSITY_START",   OPT_MAZE_DENSITY_START);
        opt_maze_density_end     = read_float("OPT_MAZE_DENSITY_END",     OPT_MAZE_DENSITY_END);
        opt_maze_reclaim_margin  = read_int  ("OPT_MAZE_RECLAIM_MARGIN",  OPT_MAZE_RECLAIM_MARGIN);
        opt_maze_time_filter     = read_bool ("OPT_MAZE_TIME_FILTER",     OPT_MAZE_TIME_FILTER);
        opt_maze_total_ticks     = read_int  ("OPT_MAZE_TOTAL_TICKS",     OPT_MAZE_TOTAL_TICKS);
        opt_maze_greedy          = read_int  ("OPT_MAZE_GREEDY",          OPT_MAZE_GREEDY);
        opt_maze_lookahead       = read_int  ("OPT_MAZE_LOOKAHEAD",       OPT_MAZE_LOOKAHEAD);
        opt_maze_lookahead_w     = read_float("OPT_MAZE_LOOKAHEAD_W",     OPT_MAZE_LOOKAHEAD_W);
        opt_maze_congestion_w    = read_float("OPT_MAZE_CONGESTION_W",    OPT_MAZE_CONGESTION_W);
        opt_maze_dynamic_fw      = read_int  ("OPT_MAZE_DYNAMIC_FW",      OPT_MAZE_DYNAMIC_FW);
        opt_maze_fw_early        = read_float("OPT_MAZE_FW_EARLY",        OPT_MAZE_FW_EARLY);
        opt_maze_fw_late         = read_float("OPT_MAZE_FW_LATE",         OPT_MAZE_FW_LATE);
        opt_maze_lookahead_topk  = read_int  ("OPT_MAZE_LOOKAHEAD_TOPK",  OPT_MAZE_LOOKAHEAD_TOPK);
        opt_maze_assign_flow     = read_bool ("OPT_MAZE_ASSIGN_FLOW",     OPT_MAZE_ASSIGN_FLOW);
        opt_maze_assign_reverse_w = read_float("OPT_MAZE_ASSIGN_REVERSE_W", OPT_MAZE_ASSIGN_REVERSE_W);
        opt_maze_assign_entry_w  = read_float("OPT_MAZE_ASSIGN_ENTRY_W",  OPT_MAZE_ASSIGN_ENTRY_W);
        opt_maze_assign_lane_w   = read_float("OPT_MAZE_ASSIGN_LANE_W",   OPT_MAZE_ASSIGN_LANE_W);
        opt_maze_assign_align_bonus = read_float("OPT_MAZE_ASSIGN_ALIGN_BONUS", OPT_MAZE_ASSIGN_ALIGN_BONUS);

        opt_maze_planner_highway = read_bool("OPT_MAZE_PLANNER_HIGHWAY", OPT_MAZE_PLANNER_HIGHWAY);
        opt_maze_planner_split   = read_bool("OPT_MAZE_PLANNER_SPLIT",   OPT_MAZE_PLANNER_SPLIT);
        opt_maze_planner_tidal   = read_bool("OPT_MAZE_PLANNER_TIDAL",   OPT_MAZE_PLANNER_TIDAL);
        opt_maze_planner_guide   = read_bool("OPT_MAZE_PLANNER_GUIDE",   OPT_MAZE_PLANNER_GUIDE);
        opt_maze_planner_lane_aware = read_bool("OPT_MAZE_PLANNER_LANE_AWARE", OPT_MAZE_PLANNER_LANE_AWARE);
        opt_maze_planner_intent_priority = read_bool("OPT_MAZE_PLANNER_INTENT_PRIORITY", OPT_MAZE_PLANNER_INTENT_PRIORITY);
        opt_maze_planner_intent_entry = read_bool("OPT_MAZE_PLANNER_INTENT_ENTRY", OPT_MAZE_PLANNER_INTENT_ENTRY);
        opt_maze_planner_intent_reverse = read_bool("OPT_MAZE_PLANNER_INTENT_REVERSE", OPT_MAZE_PLANNER_INTENT_REVERSE);
        opt_maze_planner_intent_corridor_only = read_bool("OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY", OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY);
        opt_maze_planner_entry_arbiter = read_bool("OPT_MAZE_PLANNER_ENTRY_ARBITER", OPT_MAZE_PLANNER_ENTRY_ARBITER);
        opt_maze_planner_entry_demote = read_bool("OPT_MAZE_PLANNER_ENTRY_DEMOTE", OPT_MAZE_PLANNER_ENTRY_DEMOTE);
        opt_maze_planner_entry_winner = read_bool("OPT_MAZE_PLANNER_ENTRY_WINNER", OPT_MAZE_PLANNER_ENTRY_WINNER);
        opt_maze_planner_tidal_majority = read_bool("OPT_MAZE_PLANNER_TIDAL_MAJORITY", OPT_MAZE_PLANNER_TIDAL_MAJORITY);
        opt_maze_planner_highway_penalty = read_int("OPT_MAZE_PLANNER_HIGHWAY_PENALTY", OPT_MAZE_PLANNER_HIGHWAY_PENALTY);
        opt_maze_planner_split_penalty = read_int("OPT_MAZE_PLANNER_SPLIT_PENALTY", OPT_MAZE_PLANNER_SPLIT_PENALTY);
        opt_maze_planner_tidal_period = read_int("OPT_MAZE_PLANNER_TIDAL_PERIOD", OPT_MAZE_PLANNER_TIDAL_PERIOD);
        opt_maze_planner_tidal_delta = read_float("OPT_MAZE_PLANNER_TIDAL_DELTA", OPT_MAZE_PLANNER_TIDAL_DELTA);
        opt_maze_planner_lane_flow_penalty = read_int("OPT_MAZE_PLANNER_LANE_FLOW_PENALTY", OPT_MAZE_PLANNER_LANE_FLOW_PENALTY);
        opt_maze_planner_lane_headon_penalty = read_int("OPT_MAZE_PLANNER_LANE_HEADON_PENALTY", OPT_MAZE_PLANNER_LANE_HEADON_PENALTY);
        opt_maze_planner_lane_reverse_penalty = read_int("OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY", OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY);
        opt_maze_planner_lane_wait_penalty = read_int("OPT_MAZE_PLANNER_LANE_WAIT_PENALTY", OPT_MAZE_PLANNER_LANE_WAIT_PENALTY);
        opt_maze_planner_lane_refuge_bonus = read_int("OPT_MAZE_PLANNER_LANE_REFUGE_BONUS", OPT_MAZE_PLANNER_LANE_REFUGE_BONUS);
        opt_maze_planner_intent_entry_min = read_int("OPT_MAZE_PLANNER_INTENT_ENTRY_MIN", OPT_MAZE_PLANNER_INTENT_ENTRY_MIN);
        opt_maze_planner_intent_delta = read_float("OPT_MAZE_PLANNER_INTENT_DELTA", OPT_MAZE_PLANNER_INTENT_DELTA);
        opt_maze_planner_entry_min = read_int("OPT_MAZE_PLANNER_ENTRY_MIN", OPT_MAZE_PLANNER_ENTRY_MIN);
        opt_maze_planner_entry_delta = read_float("OPT_MAZE_PLANNER_ENTRY_DELTA", OPT_MAZE_PLANNER_ENTRY_DELTA);

        // IronZone 专属
        iron_zone_rows       = read_int  ("IRON_ZONE_ROWS",       IRON_ZONE_ROWS);
        iron_zone_cols       = read_int  ("IRON_ZONE_COLS",       IRON_ZONE_COLS);
        iron_assign_variant  = read_int  ("IRON_ASSIGN_VARIANT",  IRON_ASSIGN_VARIANT);
        iron_local_radius    = read_int  ("IRON_LOCAL_RADIUS",    IRON_LOCAL_RADIUS);
        iron_max_radius      = read_int  ("IRON_MAX_RADIUS",      IRON_MAX_RADIUS);
        iron_candidate_topk  = read_int  ("IRON_CANDIDATE_TOPK",  IRON_CANDIDATE_TOPK);
        iron_dist_weight     = read_float("IRON_DIST_WEIGHT",     IRON_DIST_WEIGHT);
        iron_task_len_weight = read_float("IRON_TASK_LEN_WEIGHT", IRON_TASK_LEN_WEIGHT);
        iron_cross_zone_w    = read_float("IRON_CROSS_ZONE_W",    IRON_CROSS_ZONE_W);
        iron_endpoint_w      = read_float("IRON_ENDPOINT_W",      IRON_ENDPOINT_W);
        iron_age_weight      = read_float("IRON_AGE_WEIGHT",      IRON_AGE_WEIGHT);
        iron_assign_ratio    = read_float("IRON_ASSIGN_RATIO",    IRON_ASSIGN_RATIO);
        iron_balance_w       = read_float("IRON_BALANCE_W",       IRON_BALANCE_W);
        iron_migration_w     = read_float("IRON_MIGRATION_W",     IRON_MIGRATION_W);
        iron_balance_cap     = read_int  ("IRON_BALANCE_CAP",     IRON_BALANCE_CAP);
        iron_short_task_topk = read_int  ("IRON_SHORT_TASK_TOPK", IRON_SHORT_TASK_TOPK);
        iron_starved_task_topk = read_int("IRON_STARVED_TASK_TOPK", IRON_STARVED_TASK_TOPK);
        iron_starved_zone_threshold = read_int("IRON_STARVED_ZONE_THRESHOLD", IRON_STARVED_ZONE_THRESHOLD);
        iron_long_task_th    = read_int  ("IRON_LONG_TASK_TH",    IRON_LONG_TASK_TH);
        iron_stats_interval  = read_int  ("IRON_STATS_INTERVAL",  IRON_STATS_INTERVAL);
        iron_early_phase_ticks = read_int  ("IRON_EARLY_PHASE_TICKS", IRON_EARLY_PHASE_TICKS);
        iron_early_dist_w      = read_float("IRON_EARLY_DIST_W",      IRON_EARLY_DIST_W);
        iron_early_cross_w     = read_float("IRON_EARLY_CROSS_W",     IRON_EARLY_CROSS_W);
        iron_early_long_w      = read_float("IRON_EARLY_LONG_W",      IRON_EARLY_LONG_W);
        iron_aggressive_assign = read_bool ("IRON_AGGRESSIVE_ASSIGN", IRON_AGGRESSIVE_ASSIGN);
        iron_aggressive_iters  = read_int  ("IRON_AGGRESSIVE_ITERS",  IRON_AGGRESSIVE_ITERS);
        iron_aggressive_probe  = read_int  ("IRON_AGGRESSIVE_PROBE",  IRON_AGGRESSIVE_PROBE);
        iron_aggressive_replace_margin =
            read_float("IRON_AGGRESSIVE_REPLACE_MARGIN", IRON_AGGRESSIVE_REPLACE_MARGIN);
        iron_opportunity_ratio = read_float("IRON_OPPORTUNITY_RATIO", IRON_OPPORTUNITY_RATIO);
        iron_opportunity_dist = read_int("IRON_OPPORTUNITY_DIST", IRON_OPPORTUNITY_DIST);
        iron_opportunity_completion =
            read_int("IRON_OPPORTUNITY_COMPLETION", IRON_OPPORTUNITY_COMPLETION);
        iron_opportunity_cross = read_int("IRON_OPPORTUNITY_CROSS", IRON_OPPORTUNITY_CROSS);

        // Iron 红绿灯
        iron_lane_enabled      = read_int  ("IRON_LANE_ENABLED",       IRON_LANE_ENABLED);
        iron_lane_guard_dist   = read_int  ("IRON_LANE_GUARD_DIST",    IRON_LANE_GUARD_DIST);
        iron_lane_core_flip_th = read_int  ("IRON_LANE_CORE_FLIP_TH",  IRON_LANE_CORE_FLIP_TH);
        iron_lane_flow_ratio_flip = read_int("IRON_LANE_FLOW_RATIO_FLIP", IRON_LANE_FLOW_RATIO_FLIP);
        iron_lane_vote_period    = read_int("IRON_LANE_VOTE_PERIOD",    IRON_LANE_VOTE_PERIOD);
        iron_lane_idle_bonus   = read_float("IRON_LANE_IDLE_BONUS",    IRON_LANE_IDLE_BONUS);
        iron_lane_debug        = read_int  ("IRON_LANE_DEBUG",         IRON_LANE_DEBUG);
        iron_lane_lru_sort     = read_int  ("IRON_LANE_LRU_SORT",      IRON_LANE_LRU_SORT);

        // Iron future flow
        iron_future_flow_weight = read_float("IRON_FUTURE_FLOW_WEIGHT", IRON_FUTURE_FLOW_WEIGHT);
        iron_future_flow_k      = read_float("IRON_FUTURE_FLOW_K",      IRON_FUTURE_FLOW_K);
        iron_future_flow_decay  = read_float("IRON_FUTURE_FLOW_DECAY",  IRON_FUTURE_FLOW_DECAY);
        iron_future_flow_idle1  = read_float("IRON_FUTURE_FLOW_IDLE1",  IRON_FUTURE_FLOW_IDLE1);
        iron_future_flow_idle2  = read_float("IRON_FUTURE_FLOW_IDLE2",  IRON_FUTURE_FLOW_IDLE2);
        iron_future_flow_busy1  = read_float("IRON_FUTURE_FLOW_BUSY1",  IRON_FUTURE_FLOW_BUSY1);
        iron_future_flow_predict= read_int  ("IRON_FUTURE_FLOW_PREDICT",IRON_FUTURE_FLOW_PREDICT);
        iron_future_flow_reward_w = read_float("IRON_FUTURE_FLOW_REWARD_W", IRON_FUTURE_FLOW_REWARD_W);
        iron_future_flow_penalty_w = read_float("IRON_FUTURE_FLOW_PENALTY_W", IRON_FUTURE_FLOW_PENALTY_W);
        iron_future_flow_lock_w    = read_float("IRON_FUTURE_FLOW_LOCK_W",    IRON_FUTURE_FLOW_LOCK_W);
        iron_future_flow_lock_th   = read_float("IRON_FUTURE_FLOW_LOCK_TH",   IRON_FUTURE_FLOW_LOCK_TH);

        iron_future_flow_zone_rows = read_int("IRON_FUTURE_FLOW_ZONE_ROWS", IRON_FUTURE_FLOW_ZONE_ROWS);
        iron_future_flow_zone_cols = read_int("IRON_FUTURE_FLOW_ZONE_COLS", IRON_FUTURE_FLOW_ZONE_COLS);

        // GameOpt 专属
        game_dist_weight = read_float("GAME_DIST_WEIGHT",   GAME_DIST_WEIGHT);
        game_rebuild_pct = read_int  ("GAME_REBUILD_PCT",   GAME_REBUILD_PCT);
        game_lns_pct     = read_int  ("GAME_LNS_PCT",       GAME_LNS_PCT);
        game_sort_k      = read_int  ("GAME_SORT_K",        GAME_SORT_K);
        game_reassign    = read_bool ("GAME_REASSIGN",      GAME_REASSIGN);
        game_keep_bias   = read_float("GAME_KEEP_BIAS",     GAME_KEEP_BIAS);
        game_min_dist    = read_int  ("GAME_MIN_DIST",      GAME_MIN_DIST);
        game_cong_decay  = read_float("GAME_CONGESTION_DECAY", GAME_CONG_DECAY);
        game_grid_cell   = read_int  ("GAME_GRID_CELL",     GAME_GRID_CELL);
        game_grid_radius = read_int  ("GAME_GRID_RADIUS",   GAME_GRID_RADIUS);
        game_cong_weight = read_float("GAME_CONG_WEIGHT",   GAME_CONG_WEIGHT);
        game_dist_pow    = read_float("GAME_DIST_POW",      GAME_DIST_POW);
        game_dest_cong_weight = read_float("GAME_DEST_CONG_WEIGHT", GAME_DEST_CONG_WEIGHT);
        game_dest_cong_cap    = read_float("GAME_DEST_CONG_CAP",    GAME_DEST_CONG_CAP);
        game_age_weight       = read_float("GAME_AGE_WEIGHT",       GAME_AGE_WEIGHT);
        game_comp_weight      = read_float("GAME_COMP_WEIGHT",      GAME_COMP_WEIGHT);
        game_total_steps      = read_int  ("GAME_TOTAL_STEPS",      GAME_TOTAL_STEPS);
        game_assign_ratio     = read_float("GAME_ASSIGN_RATIO",     GAME_ASSIGN_RATIO);
        game_idle_dist        = read_int  ("GAME_IDLE_DIST",        GAME_IDLE_DIST);
        // 新版中心线参数
        game_zone_hard_cap    = read_int  ("GAME_ZONE_HARD_CAP",    GAME_ZONE_HARD_CAP);
        game_exempt_main      = read_int  ("GAME_EXEMPT_MAIN",      GAME_EXEMPT_MAIN);
        game_density_cap      = read_int  ("GAME_DENSITY_CAP",      GAME_DENSITY_CAP);
        game_dir_threshold    = read_int  ("GAME_DIR_THRESHOLD",    GAME_DIR_THRESHOLD);
        game_route_penalty    = read_int  ("GAME_ROUTE_PENALTY",    GAME_ROUTE_PENALTY);
        game_ex_main_cap      = read_int  ("GAME_EX_MAIN_CAP",     GAME_EX_MAIN_CAP);
        game_ex_nonmain_cap   = read_int  ("GAME_EX_NONMAIN_CAP",  GAME_EX_NONMAIN_CAP);
        game_bal_assign_ratio = read_float("GAME_BAL_ASSIGN_RATIO", GAME_BAL_ASSIGN_RATIO);
        // 已验证有效
        game_width_aware      = read_bool ("GAME_WIDTH_AWARE",      GAME_WIDTH_AWARE);
        game_narrow_threshold = read_int  ("GAME_NARROW_THRESHOLD", GAME_NARROW_THRESHOLD);
        game_narrow_cap_ratio = read_float("GAME_NARROW_CAP_RATIO", GAME_NARROW_CAP_RATIO);
        game_pibt_counter_flow = read_bool("GAME_PIBT_COUNTER_FLOW",GAME_PIBT_COUNTER_FLOW);
        game_counter_flow_cost = read_int ("GAME_COUNTER_FLOW_COST",GAME_COUNTER_FLOW_COST);
        game_urgency_priority  = read_bool ("GAME_URGENCY_PRIORITY", GAME_URGENCY_PRIORITY);
        game_urgency_weight    = read_float("GAME_URGENCY_WEIGHT",   GAME_URGENCY_WEIGHT);
        game_urgency_threshold = read_int  ("GAME_URGENCY_THRESHOLD",GAME_URGENCY_THRESHOLD);
        // 有效自适应策略
        game_search_radius     = read_int  ("GAME_SEARCH_RADIUS",    GAME_SEARCH_RADIUS);
        game_late_search_radius= read_int  ("GAME_LATE_SEARCH_RADIUS",GAME_LATE_SEARCH_RADIUS);
        game_late_phase_ratio  = read_float("GAME_LATE_PHASE_RATIO", GAME_LATE_PHASE_RATIO);
        game_late_relax        = read_bool ("GAME_LATE_RELAX",       GAME_LATE_RELAX);
        game_late_density_cap  = read_int  ("GAME_LATE_DENSITY_CAP", GAME_LATE_DENSITY_CAP);
        game_late_zone_cap     = read_int  ("GAME_LATE_ZONE_CAP",    GAME_LATE_ZONE_CAP);
        game_age_sort          = read_bool ("GAME_AGE_SORT",         GAME_AGE_SORT);
        game_age_sort_weight   = read_float("GAME_AGE_SORT_WEIGHT",  GAME_AGE_SORT_WEIGHT);
        game_narrow_pibt_idea = read_int("GAME_NARROW_PIBT_IDEA", GAME_NARROW_PIBT_IDEA);
        game_narrow_wait_bonus = read_int("GAME_NARROW_WAIT_BONUS", GAME_NARROW_WAIT_BONUS);
        game_narrow_density_threshold = read_int("GAME_NARROW_DENSITY_THRESHOLD", GAME_NARROW_DENSITY_THRESHOLD);
        game_narrow_base_counter_pct = read_int("GAME_NARROW_BASE_COUNTER_PCT", GAME_NARROW_BASE_COUNTER_PCT);
        game_narrow_late_start = read_int("GAME_NARROW_LATE_START", GAME_NARROW_LATE_START);
        game_narrow_late_extra = read_int("GAME_NARROW_LATE_EXTRA", GAME_NARROW_LATE_EXTRA);

        precompute_heuristic = read_bool("OPT_PRECOMPUTE_HEURISTIC", PRECOMPUTE_HEURISTIC);
        max_precompute_cells = read_int("OPT_MAX_PRECOMPUTE_CELLS", MAX_PRECOMPUTE_CELLS);
        turn_aware_heuristic = read_bool("OPT_TURN_AWARE_HEURISTIC", TURN_AWARE_HEURISTIC);

        origin_append_steps = read_int("OPT_ORIGIN_APPEND_STEPS", ORIGIN_APPEND_STEPS);
        congestion_adaptive_append = read_bool("OPT_CONGESTION_ADAPTIVE_APPEND", CONGESTION_ADAPTIVE_APPEND);

        if (debug_level >= 1) {
            std::cerr << "[optsvr] schedule_mode=" << schedule_mode
                      << " planner_mode=" << planner_mode << "\n";
        }
    }

    // 仅在环境变量存在时覆盖当前值（用于 MapTuner 之后的最终覆盖）
    void override_from_env() {
        if (std::getenv("OPT_DEBUG_LEVEL"))   debug_level   = read_int("OPT_DEBUG_LEVEL", debug_level);
        if (std::getenv("OPT_NUM_THREADS"))   num_threads   = read_int("OPT_NUM_THREADS", num_threads);
        if (std::getenv("OPT_SCHEDULE_MODE")) schedule_mode = read_int("OPT_SCHEDULE_MODE", schedule_mode);
        if (std::getenv("OPT_EXECUTOR_MODE")) executor_mode = read_int("OPT_EXECUTOR_MODE", executor_mode);
        if (std::getenv("OPT_PLANNER_MODE"))  planner_mode  = read_int("OPT_PLANNER_MODE", planner_mode);
        if (std::getenv("OPT_SKIP_TEST"))     skip_test     = read_bool("OPT_SKIP_TEST", skip_test);
        if (std::getenv("OPT_PRECOMPUTE_HEURISTIC")) precompute_heuristic = read_bool("OPT_PRECOMPUTE_HEURISTIC", precompute_heuristic);
        if (std::getenv("OPT_MAX_PRECOMPUTE_CELLS")) max_precompute_cells = read_int("OPT_MAX_PRECOMPUTE_CELLS", max_precompute_cells);
        if (std::getenv("OPT_TURN_AWARE_HEURISTIC")) turn_aware_heuristic = read_bool("OPT_TURN_AWARE_HEURISTIC", turn_aware_heuristic);
        if (std::getenv("OPT_SCORE_MODE"))    score_mode    = read_int("OPT_SCORE_MODE", score_mode);
        if (std::getenv("OPT_HEAP_DIST_WEIGHT"))  heap_dist_weight  = read_float("OPT_HEAP_DIST_WEIGHT", heap_dist_weight);
        if (std::getenv("OPT_HEAP_MAX_ASSIGN"))   heap_max_assign   = read_float("OPT_HEAP_MAX_ASSIGN", heap_max_assign);
        if (std::getenv("OPT_REASSIGN_ENABLED"))  reassign_enabled  = read_bool("OPT_REASSIGN_ENABLED", reassign_enabled);
        if (std::getenv("OPT_REASSIGN_KEEP_BIAS")) reassign_keep_bias = read_float("OPT_REASSIGN_KEEP_BIAS", reassign_keep_bias);
        if (std::getenv("OPT_FLOW_DECAY"))    flow_decay    = read_float("OPT_FLOW_DECAY", flow_decay);
        if (std::getenv("OPT_WARE_DIRECT"))   ware_direct   = read_bool("OPT_WARE_DIRECT", ware_direct);
        if (std::getenv("OPT_PIBT_FORWARD_BREAK_TIE")) {
            pibt_forward_break_tie =
                read_bool("OPT_PIBT_FORWARD_BREAK_TIE", pibt_forward_break_tie);
        }
        if (std::getenv("GAME_DIST_WEIGHT"))  game_dist_weight  = read_float("GAME_DIST_WEIGHT", game_dist_weight);
        if (std::getenv("GAME_ASSIGN_RATIO")) game_assign_ratio = read_float("GAME_ASSIGN_RATIO", game_assign_ratio);
        if (std::getenv("GAME_ZONE_HARD_CAP"))    game_zone_hard_cap    = read_int("GAME_ZONE_HARD_CAP", game_zone_hard_cap);
        if (std::getenv("GAME_EXEMPT_MAIN"))      game_exempt_main      = read_int("GAME_EXEMPT_MAIN", game_exempt_main);
        if (std::getenv("GAME_DENSITY_CAP"))      game_density_cap      = read_int("GAME_DENSITY_CAP", game_density_cap);
        if (std::getenv("GAME_DIR_THRESHOLD"))    game_dir_threshold    = read_int("GAME_DIR_THRESHOLD", game_dir_threshold);
        if (std::getenv("GAME_ROUTE_PENALTY"))    game_route_penalty    = read_int("GAME_ROUTE_PENALTY", game_route_penalty);
        if (std::getenv("GAME_EX_MAIN_CAP"))      game_ex_main_cap      = read_int("GAME_EX_MAIN_CAP", game_ex_main_cap);
        if (std::getenv("GAME_EX_NONMAIN_CAP"))   game_ex_nonmain_cap   = read_int("GAME_EX_NONMAIN_CAP", game_ex_nonmain_cap);
        if (std::getenv("GAME_BAL_ASSIGN_RATIO")) game_bal_assign_ratio = read_float("GAME_BAL_ASSIGN_RATIO", game_bal_assign_ratio);
        if (std::getenv("GAME_WIDTH_AWARE"))      game_width_aware      = read_bool("GAME_WIDTH_AWARE", game_width_aware);
        if (std::getenv("GAME_NARROW_THRESHOLD")) game_narrow_threshold = read_int("GAME_NARROW_THRESHOLD", game_narrow_threshold);
        if (std::getenv("GAME_NARROW_CAP_RATIO")) game_narrow_cap_ratio = read_float("GAME_NARROW_CAP_RATIO", game_narrow_cap_ratio);
        if (std::getenv("GAME_PIBT_COUNTER_FLOW"))game_pibt_counter_flow= read_bool("GAME_PIBT_COUNTER_FLOW", game_pibt_counter_flow);
        if (std::getenv("GAME_COUNTER_FLOW_COST"))game_counter_flow_cost= read_int("GAME_COUNTER_FLOW_COST", game_counter_flow_cost);
        if (std::getenv("GAME_URGENCY_PRIORITY"))  game_urgency_priority  = read_bool("GAME_URGENCY_PRIORITY", game_urgency_priority);
        if (std::getenv("GAME_URGENCY_WEIGHT"))    game_urgency_weight    = read_float("GAME_URGENCY_WEIGHT", game_urgency_weight);
        if (std::getenv("GAME_URGENCY_THRESHOLD")) game_urgency_threshold = read_int("GAME_URGENCY_THRESHOLD", game_urgency_threshold);
        // 有效自适应策略
        if (std::getenv("GAME_SEARCH_RADIUS"))     game_search_radius     = read_int("GAME_SEARCH_RADIUS", game_search_radius);
        if (std::getenv("GAME_LATE_SEARCH_RADIUS"))game_late_search_radius= read_int("GAME_LATE_SEARCH_RADIUS", game_late_search_radius);
        if (std::getenv("GAME_LATE_PHASE_RATIO"))  game_late_phase_ratio  = read_float("GAME_LATE_PHASE_RATIO", game_late_phase_ratio);
        if (std::getenv("GAME_LATE_RELAX"))        game_late_relax        = read_bool("GAME_LATE_RELAX", game_late_relax);
        if (std::getenv("GAME_LATE_DENSITY_CAP"))  game_late_density_cap  = read_int("GAME_LATE_DENSITY_CAP", game_late_density_cap);
        if (std::getenv("GAME_LATE_ZONE_CAP"))     game_late_zone_cap     = read_int("GAME_LATE_ZONE_CAP", game_late_zone_cap);
        if (std::getenv("GAME_AGE_SORT"))          game_age_sort          = read_bool("GAME_AGE_SORT", game_age_sort);
        if (std::getenv("GAME_AGE_SORT_WEIGHT"))   game_age_sort_weight   = read_float("GAME_AGE_SORT_WEIGHT", game_age_sort_weight);
        if (std::getenv("GAME_NARROW_PIBT_IDEA")) game_narrow_pibt_idea = read_int("GAME_NARROW_PIBT_IDEA", game_narrow_pibt_idea);
        if (std::getenv("GAME_NARROW_WAIT_BONUS")) game_narrow_wait_bonus = read_int("GAME_NARROW_WAIT_BONUS", game_narrow_wait_bonus);
        if (std::getenv("GAME_NARROW_DENSITY_THRESHOLD")) game_narrow_density_threshold = read_int("GAME_NARROW_DENSITY_THRESHOLD", game_narrow_density_threshold);
        if (std::getenv("GAME_NARROW_BASE_COUNTER_PCT")) game_narrow_base_counter_pct = read_int("GAME_NARROW_BASE_COUNTER_PCT", game_narrow_base_counter_pct);
        if (std::getenv("GAME_NARROW_LATE_START")) game_narrow_late_start = read_int("GAME_NARROW_LATE_START", game_narrow_late_start);
        if (std::getenv("GAME_NARROW_LATE_EXTRA")) game_narrow_late_extra = read_int("GAME_NARROW_LATE_EXTRA", game_narrow_late_extra);
        // Room 部分分配
        if (std::getenv("ROOM_ASSIGN_AGENT_MODE")) room_assign_agent_mode = read_int("ROOM_ASSIGN_AGENT_MODE", room_assign_agent_mode);
        if (std::getenv("ROOM_ASSIGN_RATIO"))      room_assign_ratio      = read_float("ROOM_ASSIGN_RATIO", room_assign_ratio);
        if (std::getenv("ROOM_TOPK_TASKS"))        room_topk_tasks        = read_int("ROOM_TOPK_TASKS", room_topk_tasks);
        // OptRoom 专属
        if (std::getenv("OPT_ROOM_DIST_FIRST_W"))    opt_room_dist_first_w    = read_float("OPT_ROOM_DIST_FIRST_W", opt_room_dist_first_w);
        if (std::getenv("OPT_ROOM_DIST_OVERALL"))    opt_room_dist_overall    = read_float("OPT_ROOM_DIST_OVERALL", opt_room_dist_overall);
        if (std::getenv("OPT_ROOM_SHORT_TASK_W"))    opt_room_short_task_w    = read_float("OPT_ROOM_SHORT_TASK_W", opt_room_short_task_w);
        if (std::getenv("OPT_ROOM_TOTAL_TICKS"))     opt_room_total_ticks     = read_int("OPT_ROOM_TOTAL_TICKS", opt_room_total_ticks);
        if (std::getenv("OPT_ROOM_RECLAIM_MARGIN"))  opt_room_reclaim_margin  = read_int("OPT_ROOM_RECLAIM_MARGIN", opt_room_reclaim_margin);
        if (std::getenv("OPT_ROOM_ASSIGN_RATIO"))    opt_room_assign_ratio    = read_float("OPT_ROOM_ASSIGN_RATIO", opt_room_assign_ratio);
        if (std::getenv("OPT_ROOM_MATCH_MODE"))      opt_room_match_mode      = read_int("OPT_ROOM_MATCH_MODE", opt_room_match_mode);
        if (std::getenv("OPT_ROOM_TIME_SAFETY"))     opt_room_time_safety     = read_float("OPT_ROOM_TIME_SAFETY", opt_room_time_safety);
        if (std::getenv("OPT_ROOM_REASSIGN"))        opt_room_reassign        = read_int("OPT_ROOM_REASSIGN", opt_room_reassign);
        if (std::getenv("OPT_ROOM_REASSIGN_MIN_DIST")) opt_room_reassign_min_dist = read_int("OPT_ROOM_REASSIGN_MIN_DIST", opt_room_reassign_min_dist);
        // OptMaze 专属
        if (std::getenv("OPT_MAZE_CELL_SIZE"))       opt_maze_cell_size       = read_int("OPT_MAZE_CELL_SIZE", opt_maze_cell_size);
        if (std::getenv("OPT_MAZE_DIST_FIRST_W"))    opt_maze_dist_first_w    = read_float("OPT_MAZE_DIST_FIRST_W", opt_maze_dist_first_w);
        if (std::getenv("OPT_MAZE_DIST_OVERALL"))    opt_maze_dist_overall    = read_float("OPT_MAZE_DIST_OVERALL", opt_maze_dist_overall);
        if (std::getenv("OPT_MAZE_DENSITY_START"))   opt_maze_density_start   = read_float("OPT_MAZE_DENSITY_START", opt_maze_density_start);
        if (std::getenv("OPT_MAZE_DENSITY_END"))     opt_maze_density_end     = read_float("OPT_MAZE_DENSITY_END", opt_maze_density_end);
        if (std::getenv("OPT_MAZE_RECLAIM_MARGIN"))  opt_maze_reclaim_margin  = read_int("OPT_MAZE_RECLAIM_MARGIN", opt_maze_reclaim_margin);
        if (std::getenv("OPT_MAZE_TIME_FILTER"))     opt_maze_time_filter     = read_bool("OPT_MAZE_TIME_FILTER", opt_maze_time_filter);
        if (std::getenv("OPT_MAZE_TOTAL_TICKS"))     opt_maze_total_ticks     = read_int("OPT_MAZE_TOTAL_TICKS", opt_maze_total_ticks);
        if (std::getenv("OPT_MAZE_GREEDY"))          opt_maze_greedy          = read_int("OPT_MAZE_GREEDY", opt_maze_greedy);
        if (std::getenv("OPT_MAZE_LOOKAHEAD"))       opt_maze_lookahead       = read_int("OPT_MAZE_LOOKAHEAD", opt_maze_lookahead);
        if (std::getenv("OPT_MAZE_LOOKAHEAD_W"))     opt_maze_lookahead_w     = read_float("OPT_MAZE_LOOKAHEAD_W", opt_maze_lookahead_w);
        if (std::getenv("OPT_MAZE_CONGESTION_W"))    opt_maze_congestion_w    = read_float("OPT_MAZE_CONGESTION_W", opt_maze_congestion_w);
        if (std::getenv("OPT_MAZE_DYNAMIC_FW"))      opt_maze_dynamic_fw      = read_int("OPT_MAZE_DYNAMIC_FW", opt_maze_dynamic_fw);
        if (std::getenv("OPT_MAZE_FW_EARLY"))        opt_maze_fw_early        = read_float("OPT_MAZE_FW_EARLY", opt_maze_fw_early);
        if (std::getenv("OPT_MAZE_FW_LATE"))         opt_maze_fw_late         = read_float("OPT_MAZE_FW_LATE", opt_maze_fw_late);
        if (std::getenv("OPT_MAZE_LOOKAHEAD_TOPK"))  opt_maze_lookahead_topk  = read_int("OPT_MAZE_LOOKAHEAD_TOPK", opt_maze_lookahead_topk);
        if (std::getenv("OPT_MAZE_ASSIGN_FLOW"))     opt_maze_assign_flow     = read_bool("OPT_MAZE_ASSIGN_FLOW", opt_maze_assign_flow);
        if (std::getenv("OPT_MAZE_ASSIGN_REVERSE_W")) opt_maze_assign_reverse_w = read_float("OPT_MAZE_ASSIGN_REVERSE_W", opt_maze_assign_reverse_w);
        if (std::getenv("OPT_MAZE_ASSIGN_ENTRY_W"))  opt_maze_assign_entry_w  = read_float("OPT_MAZE_ASSIGN_ENTRY_W", opt_maze_assign_entry_w);
        if (std::getenv("OPT_MAZE_ASSIGN_LANE_W"))   opt_maze_assign_lane_w   = read_float("OPT_MAZE_ASSIGN_LANE_W", opt_maze_assign_lane_w);
        if (std::getenv("OPT_MAZE_ASSIGN_ALIGN_BONUS")) opt_maze_assign_align_bonus = read_float("OPT_MAZE_ASSIGN_ALIGN_BONUS", opt_maze_assign_align_bonus);

        if (std::getenv("OPT_MAZE_PLANNER_HIGHWAY")) opt_maze_planner_highway = read_bool("OPT_MAZE_PLANNER_HIGHWAY", opt_maze_planner_highway);
        if (std::getenv("OPT_MAZE_PLANNER_SPLIT"))   opt_maze_planner_split   = read_bool("OPT_MAZE_PLANNER_SPLIT", opt_maze_planner_split);
        if (std::getenv("OPT_MAZE_PLANNER_TIDAL"))   opt_maze_planner_tidal   = read_bool("OPT_MAZE_PLANNER_TIDAL", opt_maze_planner_tidal);
        if (std::getenv("OPT_MAZE_PLANNER_GUIDE"))   opt_maze_planner_guide   = read_bool("OPT_MAZE_PLANNER_GUIDE", opt_maze_planner_guide);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_AWARE")) opt_maze_planner_lane_aware = read_bool("OPT_MAZE_PLANNER_LANE_AWARE", opt_maze_planner_lane_aware);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_PRIORITY")) opt_maze_planner_intent_priority = read_bool("OPT_MAZE_PLANNER_INTENT_PRIORITY", opt_maze_planner_intent_priority);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_ENTRY")) opt_maze_planner_intent_entry = read_bool("OPT_MAZE_PLANNER_INTENT_ENTRY", opt_maze_planner_intent_entry);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_REVERSE")) opt_maze_planner_intent_reverse = read_bool("OPT_MAZE_PLANNER_INTENT_REVERSE", opt_maze_planner_intent_reverse);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY")) opt_maze_planner_intent_corridor_only = read_bool("OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY", opt_maze_planner_intent_corridor_only);
        if (std::getenv("OPT_MAZE_PLANNER_ENTRY_ARBITER")) opt_maze_planner_entry_arbiter = read_bool("OPT_MAZE_PLANNER_ENTRY_ARBITER", opt_maze_planner_entry_arbiter);
        if (std::getenv("OPT_MAZE_PLANNER_ENTRY_DEMOTE")) opt_maze_planner_entry_demote = read_bool("OPT_MAZE_PLANNER_ENTRY_DEMOTE", opt_maze_planner_entry_demote);
        if (std::getenv("OPT_MAZE_PLANNER_ENTRY_WINNER")) opt_maze_planner_entry_winner = read_bool("OPT_MAZE_PLANNER_ENTRY_WINNER", opt_maze_planner_entry_winner);
        if (std::getenv("OPT_MAZE_PLANNER_TIDAL_MAJORITY")) opt_maze_planner_tidal_majority = read_bool("OPT_MAZE_PLANNER_TIDAL_MAJORITY", opt_maze_planner_tidal_majority);
        if (std::getenv("OPT_MAZE_PLANNER_HIGHWAY_PENALTY")) opt_maze_planner_highway_penalty = read_int("OPT_MAZE_PLANNER_HIGHWAY_PENALTY", opt_maze_planner_highway_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_SPLIT_PENALTY")) opt_maze_planner_split_penalty = read_int("OPT_MAZE_PLANNER_SPLIT_PENALTY", opt_maze_planner_split_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_TIDAL_PERIOD")) opt_maze_planner_tidal_period = read_int("OPT_MAZE_PLANNER_TIDAL_PERIOD", opt_maze_planner_tidal_period);
        if (std::getenv("OPT_MAZE_PLANNER_TIDAL_DELTA")) opt_maze_planner_tidal_delta = read_float("OPT_MAZE_PLANNER_TIDAL_DELTA", opt_maze_planner_tidal_delta);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_FLOW_PENALTY")) opt_maze_planner_lane_flow_penalty = read_int("OPT_MAZE_PLANNER_LANE_FLOW_PENALTY", opt_maze_planner_lane_flow_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_HEADON_PENALTY")) opt_maze_planner_lane_headon_penalty = read_int("OPT_MAZE_PLANNER_LANE_HEADON_PENALTY", opt_maze_planner_lane_headon_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY")) opt_maze_planner_lane_reverse_penalty = read_int("OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY", opt_maze_planner_lane_reverse_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_WAIT_PENALTY")) opt_maze_planner_lane_wait_penalty = read_int("OPT_MAZE_PLANNER_LANE_WAIT_PENALTY", opt_maze_planner_lane_wait_penalty);
        if (std::getenv("OPT_MAZE_PLANNER_LANE_REFUGE_BONUS")) opt_maze_planner_lane_refuge_bonus = read_int("OPT_MAZE_PLANNER_LANE_REFUGE_BONUS", opt_maze_planner_lane_refuge_bonus);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_ENTRY_MIN")) opt_maze_planner_intent_entry_min = read_int("OPT_MAZE_PLANNER_INTENT_ENTRY_MIN", opt_maze_planner_intent_entry_min);
        if (std::getenv("OPT_MAZE_PLANNER_INTENT_DELTA")) opt_maze_planner_intent_delta = read_float("OPT_MAZE_PLANNER_INTENT_DELTA", opt_maze_planner_intent_delta);
        if (std::getenv("OPT_MAZE_PLANNER_ENTRY_MIN")) opt_maze_planner_entry_min = read_int("OPT_MAZE_PLANNER_ENTRY_MIN", opt_maze_planner_entry_min);
        if (std::getenv("OPT_MAZE_PLANNER_ENTRY_DELTA")) opt_maze_planner_entry_delta = read_float("OPT_MAZE_PLANNER_ENTRY_DELTA", opt_maze_planner_entry_delta);
        // IronZone 专属
        if (std::getenv("IRON_ZONE_ROWS"))       iron_zone_rows       = read_int  ("IRON_ZONE_ROWS", iron_zone_rows);
        if (std::getenv("IRON_ZONE_COLS"))       iron_zone_cols       = read_int  ("IRON_ZONE_COLS", iron_zone_cols);
        if (std::getenv("IRON_ASSIGN_VARIANT"))  iron_assign_variant  = read_int  ("IRON_ASSIGN_VARIANT", iron_assign_variant);
        if (std::getenv("IRON_LOCAL_RADIUS"))    iron_local_radius    = read_int  ("IRON_LOCAL_RADIUS", iron_local_radius);
        if (std::getenv("IRON_MAX_RADIUS"))      iron_max_radius      = read_int  ("IRON_MAX_RADIUS", iron_max_radius);
        if (std::getenv("IRON_CANDIDATE_TOPK"))  iron_candidate_topk  = read_int  ("IRON_CANDIDATE_TOPK", iron_candidate_topk);
        if (std::getenv("IRON_DIST_WEIGHT"))     iron_dist_weight     = read_float("IRON_DIST_WEIGHT", iron_dist_weight);
        if (std::getenv("IRON_TASK_LEN_WEIGHT")) iron_task_len_weight = read_float("IRON_TASK_LEN_WEIGHT", iron_task_len_weight);
        if (std::getenv("IRON_CROSS_ZONE_W"))    iron_cross_zone_w    = read_float("IRON_CROSS_ZONE_W", iron_cross_zone_w);
        if (std::getenv("IRON_ENDPOINT_W"))      iron_endpoint_w      = read_float("IRON_ENDPOINT_W", iron_endpoint_w);
        if (std::getenv("IRON_AGE_WEIGHT"))      iron_age_weight      = read_float("IRON_AGE_WEIGHT", iron_age_weight);
        if (std::getenv("IRON_ASSIGN_RATIO"))    iron_assign_ratio    = read_float("IRON_ASSIGN_RATIO", iron_assign_ratio);
        if (std::getenv("IRON_BALANCE_W"))       iron_balance_w       = read_float("IRON_BALANCE_W", iron_balance_w);
        if (std::getenv("IRON_MIGRATION_W"))     iron_migration_w     = read_float("IRON_MIGRATION_W", iron_migration_w);
        if (std::getenv("IRON_BALANCE_CAP"))     iron_balance_cap     = read_int  ("IRON_BALANCE_CAP", iron_balance_cap);
        if (std::getenv("IRON_SHORT_TASK_TOPK")) iron_short_task_topk = read_int  ("IRON_SHORT_TASK_TOPK", iron_short_task_topk);
        if (std::getenv("IRON_STARVED_TASK_TOPK")) iron_starved_task_topk = read_int("IRON_STARVED_TASK_TOPK", iron_starved_task_topk);
        if (std::getenv("IRON_STARVED_ZONE_THRESHOLD")) iron_starved_zone_threshold = read_int("IRON_STARVED_ZONE_THRESHOLD", iron_starved_zone_threshold);
        if (std::getenv("IRON_LONG_TASK_TH"))    iron_long_task_th    = read_int  ("IRON_LONG_TASK_TH", iron_long_task_th);
        if (std::getenv("IRON_STATS_INTERVAL"))  iron_stats_interval  = read_int  ("IRON_STATS_INTERVAL", iron_stats_interval);
        if (std::getenv("IRON_EARLY_PHASE_TICKS")) iron_early_phase_ticks = read_int  ("IRON_EARLY_PHASE_TICKS", iron_early_phase_ticks);
        if (std::getenv("IRON_EARLY_DIST_W"))      iron_early_dist_w      = read_float("IRON_EARLY_DIST_W", iron_early_dist_w);
        if (std::getenv("IRON_EARLY_CROSS_W"))     iron_early_cross_w     = read_float("IRON_EARLY_CROSS_W", iron_early_cross_w);
        if (std::getenv("IRON_EARLY_LONG_W"))      iron_early_long_w      = read_float("IRON_EARLY_LONG_W", iron_early_long_w);
        if (std::getenv("IRON_AGGRESSIVE_ASSIGN")) iron_aggressive_assign = read_bool("IRON_AGGRESSIVE_ASSIGN", iron_aggressive_assign);
        if (std::getenv("IRON_AGGRESSIVE_ITERS"))  iron_aggressive_iters  = read_int("IRON_AGGRESSIVE_ITERS", iron_aggressive_iters);
        if (std::getenv("IRON_AGGRESSIVE_PROBE"))  iron_aggressive_probe  = read_int("IRON_AGGRESSIVE_PROBE", iron_aggressive_probe);
        if (std::getenv("IRON_AGGRESSIVE_REPLACE_MARGIN")) {
            iron_aggressive_replace_margin =
                read_float("IRON_AGGRESSIVE_REPLACE_MARGIN", iron_aggressive_replace_margin);
        }
        if (std::getenv("IRON_OPPORTUNITY_RATIO")) {
            iron_opportunity_ratio = read_float("IRON_OPPORTUNITY_RATIO", iron_opportunity_ratio);
        }
        if (std::getenv("IRON_OPPORTUNITY_DIST")) {
            iron_opportunity_dist = read_int("IRON_OPPORTUNITY_DIST", iron_opportunity_dist);
        }
        if (std::getenv("IRON_OPPORTUNITY_COMPLETION")) {
            iron_opportunity_completion =
                read_int("IRON_OPPORTUNITY_COMPLETION", iron_opportunity_completion);
        }
        if (std::getenv("IRON_OPPORTUNITY_CROSS")) {
            iron_opportunity_cross = read_int("IRON_OPPORTUNITY_CROSS", iron_opportunity_cross);
        }
        // RoomPlanner / RoomExecutor 优化 (精简后仅保留有效参数)
        if (std::getenv("OPT_ROOM_FLOW_SCALE"))          opt_room_flow_scale          = read_int("OPT_ROOM_FLOW_SCALE", opt_room_flow_scale);
        if (std::getenv("OPT_ROOM_LATE_FORCE_REPLAN"))   opt_room_late_force_replan   = read_int("OPT_ROOM_LATE_FORCE_REPLAN", opt_room_late_force_replan);
        if (std::getenv("OPT_ROOM_REPLAN_START_TICK"))   opt_room_replan_start_tick   = read_int("OPT_ROOM_REPLAN_START_TICK", opt_room_replan_start_tick);
        // RoomTidalPlanner (Mode 7) 可调参数
        if (std::getenv("OPT_TIDAL_PERIOD"))             tidal_period             = read_int("OPT_TIDAL_PERIOD", tidal_period);
        if (std::getenv("OPT_TIDAL_GUIDE_BONUS"))        tidal_guide_bonus        = read_int("OPT_TIDAL_GUIDE_BONUS", tidal_guide_bonus);
        if (std::getenv("OPT_TIDAL_BACKTRACK_PENALTY"))  tidal_backtrack_penalty  = read_int("OPT_TIDAL_BACKTRACK_PENALTY", tidal_backtrack_penalty);
        if (std::getenv("OPT_TIDAL_BACKTRACK_STEP"))     tidal_backtrack_step     = read_int("OPT_TIDAL_BACKTRACK_STEP", tidal_backtrack_step);
        if (std::getenv("OPT_TIDAL_WAIT_PENALTY"))       tidal_wait_penalty       = read_int("OPT_TIDAL_WAIT_PENALTY", tidal_wait_penalty);
        if (std::getenv("OPT_TIDAL_FORWARD_SKIP"))       tidal_forward_skip       = read_int("OPT_TIDAL_FORWARD_SKIP", tidal_forward_skip);
        if (std::getenv("OPT_TIDAL_OFF_PATH"))           tidal_off_path           = read_int("OPT_TIDAL_OFF_PATH", tidal_off_path);
        if (std::getenv("OPT_TIDAL_PRIORITY_DELTA"))     tidal_priority_delta     = read_int("OPT_TIDAL_PRIORITY_DELTA", tidal_priority_delta);
        if (std::getenv("OPT_TIDAL_REFUGE_PENALTY"))     tidal_refuge_penalty     = read_int("OPT_TIDAL_REFUGE_PENALTY", tidal_refuge_penalty);
        if (std::getenv("OPT_TIDAL_PIBT_RETRY_LIMIT"))   tidal_pibt_retry_limit   = read_int("OPT_TIDAL_PIBT_RETRY_LIMIT", tidal_pibt_retry_limit);
        // Executor yield策略 (R57/R58)
        if (std::getenv("OPT_ROOM_EXEC_YIELD_HIGHWAY"))  room_exec_yield_highway  = read_int("OPT_ROOM_EXEC_YIELD_HIGHWAY", room_exec_yield_highway);
        if (std::getenv("OPT_ROOM_EXEC_YIELD_THRESH"))   room_exec_yield_thresh   = read_int("OPT_ROOM_EXEC_YIELD_THRESH", room_exec_yield_thresh);
        if (std::getenv("OPT_ROOM_PIBT_YIELD"))          room_pibt_yield          = read_int("OPT_ROOM_PIBT_YIELD", room_pibt_yield);
        if (std::getenv("OPT_ROOM_PIBT_YIELD_PENALTY"))  room_pibt_yield_penalty  = read_int("OPT_ROOM_PIBT_YIELD_PENALTY", room_pibt_yield_penalty);
        // R59: Schedule层新策略
        if (std::getenv("OPT_ROOM_LATE_FW_PHASE"))       opt_room_late_fw_phase       = read_int  ("OPT_ROOM_LATE_FW_PHASE",      opt_room_late_fw_phase);
        if (std::getenv("OPT_ROOM_LATE_FW_VALUE"))       opt_room_late_fw_value       = read_float("OPT_ROOM_LATE_FW_VALUE",      opt_room_late_fw_value);
        if (std::getenv("OPT_ROOM_LATE_FW_START_TICK"))  opt_room_late_fw_start_tick  = read_int  ("OPT_ROOM_LATE_FW_START_TICK", opt_room_late_fw_start_tick);
        if (std::getenv("OPT_ROOM_PROGRESS_BOOST"))      opt_room_progress_boost      = read_int  ("OPT_ROOM_PROGRESS_BOOST",     opt_room_progress_boost);
        if (std::getenv("OPT_ROOM_PROGRESS_BOOST_W"))    opt_room_progress_boost_w    = read_float("OPT_ROOM_PROGRESS_BOOST_W",   opt_room_progress_boost_w);
        // R59: Planner层新策略
        if (std::getenv("OPT_TIDAL_CORRIDOR_FORWARD"))        tidal_corridor_forward       = read_int("OPT_TIDAL_CORRIDOR_FORWARD",       tidal_corridor_forward);
        if (std::getenv("OPT_TIDAL_CORRIDOR_FORWARD_BONUS"))  tidal_corridor_forward_bonus = read_int("OPT_TIDAL_CORRIDOR_FORWARD_BONUS", tidal_corridor_forward_bonus);

    }

private:
    RuntimeConfig() {
        skip_test = SKIP_TEST; debug_level = DEBUG_LEVEL; num_threads = NUM_THREADS;
        schedule_mode = SCHEDULE_MODE; executor_mode = EXECUTOR_MODE; planner_mode = PLANNER_MODE;
        score_mode = SCORE_MODE;
        heap_dist_weight = HEAP_DIST_WEIGHT; heap_rebuild_pct = HEAP_REBUILD_PCT;
        heap_lns_pct = HEAP_LNS_PCT; heap_sort_k = HEAP_SORT_K;
        heap_max_assign = HEAP_MAX_ASSIGN;
        candidate_topk = CANDIDATE_TOPK; reassign_enabled = REASSIGN_ENABLED;
        reassign_keep_bias = REASSIGN_KEEP_BIAS; reassign_min_dist = REASSIGN_MIN_DIST;
        congestion_decay = CONGESTION_DECAY; difficulty_weight = DIFFICULTY_WEIGHT;
        grid_cell_size = CONGESTION_GRID_CELL; grid_radius = CONGESTION_GRID_RADIUS;
        congestion_use_history = CONGESTION_USE_HISTORY;
        flow_decay = FLOW_DECAY; flow_decay_scale = FLOW_DECAY_SCALE;
        ware_direct = WARE_DIRECT;
        pibt_forward_break_tie = OPT_PIBT_FORWARD_BREAK_TIE;
        guide_flow_opposite_w = GUIDE_FLOW_OPPOSITE_W;
        guide_flow_same_w     = GUIDE_FLOW_SAME_W;

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
        game_narrow_pibt_idea = GAME_NARROW_PIBT_IDEA;
        game_narrow_wait_bonus = GAME_NARROW_WAIT_BONUS;
        game_narrow_density_threshold = GAME_NARROW_DENSITY_THRESHOLD;
        game_narrow_base_counter_pct = GAME_NARROW_BASE_COUNTER_PCT;
        game_narrow_late_start = GAME_NARROW_LATE_START;
        game_narrow_late_extra = GAME_NARROW_LATE_EXTRA;

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

        opt_maze_cell_size = OPT_MAZE_CELL_SIZE;
        opt_maze_dist_first_w = OPT_MAZE_DIST_FIRST_W;
        opt_maze_dist_overall = OPT_MAZE_DIST_OVERALL;
        opt_maze_density_start = OPT_MAZE_DENSITY_START;
        opt_maze_density_end = OPT_MAZE_DENSITY_END;
        opt_maze_reclaim_margin = OPT_MAZE_RECLAIM_MARGIN;
        opt_maze_time_filter = OPT_MAZE_TIME_FILTER;
        opt_maze_total_ticks = OPT_MAZE_TOTAL_TICKS;
        opt_maze_greedy = OPT_MAZE_GREEDY;
        opt_maze_lookahead = OPT_MAZE_LOOKAHEAD;
        opt_maze_lookahead_w = OPT_MAZE_LOOKAHEAD_W;
        opt_maze_congestion_w = OPT_MAZE_CONGESTION_W;
        opt_maze_dynamic_fw = OPT_MAZE_DYNAMIC_FW;
        opt_maze_fw_early = OPT_MAZE_FW_EARLY;
        opt_maze_fw_late = OPT_MAZE_FW_LATE;
        opt_maze_lookahead_topk = OPT_MAZE_LOOKAHEAD_TOPK;
        opt_maze_assign_flow = OPT_MAZE_ASSIGN_FLOW;
        opt_maze_assign_reverse_w = OPT_MAZE_ASSIGN_REVERSE_W;
        opt_maze_assign_entry_w = OPT_MAZE_ASSIGN_ENTRY_W;
        opt_maze_assign_lane_w = OPT_MAZE_ASSIGN_LANE_W;
        opt_maze_assign_align_bonus = OPT_MAZE_ASSIGN_ALIGN_BONUS;

        opt_maze_planner_highway = OPT_MAZE_PLANNER_HIGHWAY;
        opt_maze_planner_split = OPT_MAZE_PLANNER_SPLIT;
        opt_maze_planner_tidal = OPT_MAZE_PLANNER_TIDAL;
        opt_maze_planner_guide = OPT_MAZE_PLANNER_GUIDE;
        opt_maze_planner_lane_aware = OPT_MAZE_PLANNER_LANE_AWARE;
        opt_maze_planner_intent_priority = OPT_MAZE_PLANNER_INTENT_PRIORITY;
        opt_maze_planner_intent_entry = OPT_MAZE_PLANNER_INTENT_ENTRY;
        opt_maze_planner_intent_reverse = OPT_MAZE_PLANNER_INTENT_REVERSE;
        opt_maze_planner_intent_corridor_only = OPT_MAZE_PLANNER_INTENT_CORRIDOR_ONLY;
        opt_maze_planner_entry_arbiter = OPT_MAZE_PLANNER_ENTRY_ARBITER;
        opt_maze_planner_entry_demote = OPT_MAZE_PLANNER_ENTRY_DEMOTE;
        opt_maze_planner_entry_winner = OPT_MAZE_PLANNER_ENTRY_WINNER;
        opt_maze_planner_tidal_majority = OPT_MAZE_PLANNER_TIDAL_MAJORITY;
        opt_maze_planner_highway_penalty = OPT_MAZE_PLANNER_HIGHWAY_PENALTY;
        opt_maze_planner_split_penalty = OPT_MAZE_PLANNER_SPLIT_PENALTY;
        opt_maze_planner_tidal_period = OPT_MAZE_PLANNER_TIDAL_PERIOD;
        opt_maze_planner_tidal_delta = OPT_MAZE_PLANNER_TIDAL_DELTA;
        opt_maze_planner_lane_flow_penalty = OPT_MAZE_PLANNER_LANE_FLOW_PENALTY;
        opt_maze_planner_lane_headon_penalty = OPT_MAZE_PLANNER_LANE_HEADON_PENALTY;
        opt_maze_planner_lane_reverse_penalty = OPT_MAZE_PLANNER_LANE_REVERSE_PENALTY;
        opt_maze_planner_lane_wait_penalty = OPT_MAZE_PLANNER_LANE_WAIT_PENALTY;
        opt_maze_planner_lane_refuge_bonus = OPT_MAZE_PLANNER_LANE_REFUGE_BONUS;
        opt_maze_planner_intent_entry_min = OPT_MAZE_PLANNER_INTENT_ENTRY_MIN;
        opt_maze_planner_intent_delta = OPT_MAZE_PLANNER_INTENT_DELTA;
        opt_maze_planner_entry_min = OPT_MAZE_PLANNER_ENTRY_MIN;
        opt_maze_planner_entry_delta = OPT_MAZE_PLANNER_ENTRY_DELTA;

        iron_zone_rows = IRON_ZONE_ROWS;
        iron_zone_cols = IRON_ZONE_COLS;
        iron_assign_variant = IRON_ASSIGN_VARIANT;
        iron_local_radius = IRON_LOCAL_RADIUS;
        iron_max_radius = IRON_MAX_RADIUS;
        iron_candidate_topk = IRON_CANDIDATE_TOPK;
        iron_dist_weight = IRON_DIST_WEIGHT;
        iron_task_len_weight = IRON_TASK_LEN_WEIGHT;
        iron_cross_zone_w = IRON_CROSS_ZONE_W;
        iron_endpoint_w = IRON_ENDPOINT_W;
        iron_age_weight = IRON_AGE_WEIGHT;
        iron_assign_ratio = IRON_ASSIGN_RATIO;
        iron_balance_w = IRON_BALANCE_W;
        iron_migration_w = IRON_MIGRATION_W;
        iron_balance_cap = IRON_BALANCE_CAP;
        iron_short_task_topk = IRON_SHORT_TASK_TOPK;
        iron_starved_task_topk = IRON_STARVED_TASK_TOPK;
        iron_starved_zone_threshold = IRON_STARVED_ZONE_THRESHOLD;
        iron_long_task_th = IRON_LONG_TASK_TH;
        iron_stats_interval = IRON_STATS_INTERVAL;
        iron_early_phase_ticks = IRON_EARLY_PHASE_TICKS;
        iron_early_dist_w = IRON_EARLY_DIST_W;
        iron_early_cross_w = IRON_EARLY_CROSS_W;
        iron_early_long_w = IRON_EARLY_LONG_W;
        iron_aggressive_assign = IRON_AGGRESSIVE_ASSIGN;
        iron_aggressive_iters = IRON_AGGRESSIVE_ITERS;
        iron_aggressive_probe = IRON_AGGRESSIVE_PROBE;
        iron_aggressive_replace_margin = IRON_AGGRESSIVE_REPLACE_MARGIN;
        iron_opportunity_ratio = IRON_OPPORTUNITY_RATIO;
        iron_opportunity_dist = IRON_OPPORTUNITY_DIST;
        iron_opportunity_completion = IRON_OPPORTUNITY_COMPLETION;
        iron_opportunity_cross = IRON_OPPORTUNITY_CROSS;

        iron_lane_enabled      = IRON_LANE_ENABLED;
        iron_lane_guard_dist   = IRON_LANE_GUARD_DIST;
        iron_lane_core_flip_th = IRON_LANE_CORE_FLIP_TH;
        iron_lane_flow_ratio_flip = IRON_LANE_FLOW_RATIO_FLIP;
        iron_lane_vote_period    = IRON_LANE_VOTE_PERIOD;
        iron_lane_idle_bonus   = IRON_LANE_IDLE_BONUS;
        iron_lane_debug        = IRON_LANE_DEBUG;
        iron_lane_lru_sort     = IRON_LANE_LRU_SORT;

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
        tidal_priority_delta = OPT_TIDAL_PRIORITY_DELTA;
        tidal_refuge_penalty = OPT_TIDAL_REFUGE_PENALTY;
        tidal_pibt_retry_limit = OPT_TIDAL_PIBT_RETRY_LIMIT;
        // Executor yield策略 (R57/R58)
        room_exec_yield_highway = OPT_ROOM_EXEC_YIELD_HIGHWAY;
        room_exec_yield_thresh = OPT_ROOM_EXEC_YIELD_THRESH;
        room_pibt_yield = OPT_ROOM_PIBT_YIELD;
        room_pibt_yield_penalty = OPT_ROOM_PIBT_YIELD_PENALTY;

        // R59: Schedule层新策略
        opt_room_late_fw_phase = OPT_ROOM_LATE_FW_PHASE;
        opt_room_late_fw_value = OPT_ROOM_LATE_FW_VALUE;
        opt_room_late_fw_start_tick = OPT_ROOM_LATE_FW_START_TICK;
        opt_room_progress_boost = OPT_ROOM_PROGRESS_BOOST;
        opt_room_progress_boost_w = OPT_ROOM_PROGRESS_BOOST_W;
        // R59: Planner层新策略
        tidal_corridor_forward = OPT_TIDAL_CORRIDOR_FORWARD;
        tidal_corridor_forward_bonus = OPT_TIDAL_CORRIDOR_FORWARD_BONUS;

        precompute_heuristic = PRECOMPUTE_HEURISTIC;
        max_precompute_cells = MAX_PRECOMPUTE_CELLS;
        turn_aware_heuristic = TURN_AWARE_HEURISTIC;
        origin_append_steps = ORIGIN_APPEND_STEPS;
        congestion_adaptive_append = CONGESTION_ADAPTIVE_APPEND;
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
