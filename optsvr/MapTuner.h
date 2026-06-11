// optsvr/MapTuner.h — 地图类型识别 + 自动设置 schedule_mode
#pragma once

#include "RuntimeConfig.h"
#include "MapProfile.h"
#include <cstdlib>
#include <iostream>

namespace optsvr {

// 根据地图类型自动调整 RuntimeConfig
inline void apply_map_tuning(const MapProfile& profile, const SharedEnvironment* /*env*/) {
    auto& c = cfg();

    switch (profile.type) {

        case MapType::MAZE_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 6;    // OptMazeScheduler
            c.executor_mode = 3;    // DHExecutor
            c.planner_mode = 0;     // Default planner; MZ-R3 I4 is current stable MAZE default.

            // MZ-R3 I4 stable submit line: 372/364 on maze_40 3000tick.
            c.opt_maze_dist_first_w = 11.0f;
            c.opt_maze_dist_overall = 0.7f;

            break;

        case MapType::ROOM_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 7;    // OptRoomScheduler
            c.executor_mode = 5;    // RoomExecutor
            c.planner_mode = 7;     // RoomTidalPlanner (潮汐门控)
            // 最优参数 (验证: delta500+refuge60+retry3 → R56-IDEA6 peak=766, avg=742 top1)
            c.opt_room_assign_ratio   = 0.80f;
            c.opt_room_time_safety    = 1.2f;
            c.opt_room_match_mode     = 1;      // greedy匹配
            c.opt_room_dist_first_w   = 2.0f;
            // TidalPlanner 参数 (各参数均经3次独立运行验证)
            c.tidal_period            = 50;
            c.tidal_off_path          = 15;
            c.tidal_backtrack_penalty = 60;
            c.tidal_backtrack_step    = 12;
            c.tidal_guide_bonus       = 30;
            c.tidal_wait_penalty      = 12;
            c.tidal_forward_skip      = 4;
            c.tidal_priority_delta    = 500;   // R52 baseline peak=759, avg=749
            c.tidal_refuge_penalty    = 60;    // R51 refuge60 avg=736, peak=743
            c.tidal_pibt_retry_limit  = 3;     // R56 pibt_retry3 peak=766, avg=742 TOP1
            break;

        case MapType::WAREHOUSE_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 1;
            c.ware_direct = true;
            break;

        case MapType::CITY_DOMAIN:
            c.skip_test = true;
            c.planner_mode = 4;
            break;

        case MapType::GAME_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 8;  // GameBalanceOptScheduler baseline
            c.executor_mode = 6;  // GameExecutor (TPG + 连续STOP截断)
            c.planner_mode  = 8;  // GameNarrowPIBTPlanner

            c.reassign_enabled = false;
            c.game_narrow_pibt_idea = 0;         // baseline: no scheduled IDEA by default
            c.game_narrow_wait_bonus = 2;
            c.game_narrow_density_threshold = 12;
            c.game_narrow_base_counter_pct = 100;
            c.game_narrow_late_start = 2500;
            c.game_narrow_late_extra = 1;
            // 已验证最优参数
            // c.game_width_aware       = true;   // 瓶颈宽度感知容量控制
            // c.game_narrow_threshold  = 4;      // 宽度<=4视为窄瓶颈
            // c.game_narrow_cap_ratio  = 0.5f;   // 窄瓶颈容量 = width * 0.5
            // c.game_pibt_counter_flow = true;   // PIBT逆流惩罚
            // c.game_counter_flow_cost = 20;     // 逆流额外代价
            // c.game_urgency_priority  = true;   // 紧迫度优先级
            // c.game_urgency_weight    = 10.0f;  // urgency weight
            // c.game_urgency_threshold = 50;     // urgency 阈值
            // c.game_search_radius     = 1;      // 默认搜索半径
            // c.game_late_search_radius = 3;     // 后期搜索半径
            // c.game_late_phase_ratio  = 0.4f;   // 40%后进入后期模式
            break;

        case MapType::IRON_HARVEST_DOMAIN:
            c.skip_test = false;
            c.schedule_mode = 10;  // IronZoneScheduler
            c.executor_mode = 0;
            c.precompute_heuristic = true;

            c.reassign_enabled=false;
            break;

        case MapType::RANDOM_DOMAIN:
            c.skip_test = true;
            c.schedule_mode = 2;
            c.planner_mode = 4;     // EPIBT
            break;

        case MapType::UNKNOWN:
            c.skip_test = true;
            break;
    }

    // 环境变量最高优先级：仅在设置时覆盖 MapTuner 的选择
    c.override_from_env();


    if (c.skip_test) {
        std::cerr << "[MapTuner] SKIP_TEST for: " << profile.type_to_string() << "\n";
    } else if (c.debug_level >= 1) {
        std::cerr << "[MapTuner] " << profile.type_to_string()
                  << " → schedule_mode=" << c.schedule_mode << "\n";
    }
}

} // namespace optsvr
