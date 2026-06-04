#pragma once
// Shared state between scheduler (Centerline) and PIBT for direction ahead-check

#include "../src/config.h"
#include <vector>

namespace PIBT_Lock {
    static constexpr int MAX_AGENTS = 10000;

    extern int g_agent_blocked[MAX_AGENTS];    // 1 = PIBT should wait, 0 = normal
    extern int g_parked_task[MAX_AGENTS];      // pending second errand task id, -1 = none
}

// Lane direction lock state — set by my_planner.cpp vote, read by my_pibt.cpp forbids/pibt_priority
namespace LaneFlow {
    extern int g_lane_lock_dir[20];             // 每条lane的锁方向: +1=end0→end1, -1=end1→end0, 0=无锁
    extern int g_lane_next_vote_tick;          // 当前会话中下一次投票的tick
    extern int* g_lane_exit_target;            // [loc] = 每个lane格最近的出口(非lane格)，-1=未初始化
    extern bool g_lane_exit_target_initialized;
    extern int g_lane_blocked_plus[20];          // 当前周期被拦(+1方向)次数
    extern int g_lane_blocked_minus[20];         // 当前周期被拦(-1方向)次数
    extern int g_lane_passed_plus[20];           // 当前周期放行(+1方向)次数
    extern int g_lane_passed_minus[20];          // 当前周期放行(-1方向)次数
}

// Centerline data shared with PIBT for wall-hugging (set by scheduler after init)
namespace CenterlineAccess {
    extern const std::vector<int>* g_branch_dist;     // [loc]
    extern const std::vector<int>* g_main_dist;       // [loc]
    extern const std::vector<int>* g_zone_is_main;    // [zone_id]
    extern const std::vector<int>* g_mp_loc;          // [si] = main point map location
    extern const std::vector<int>* g_mp_main_dist;    // [si] = main_dist at main point
    extern int g_mp_count;
    extern int g_n24c;
    extern int g_n24r;
    extern float active_ratio;       // [0-1] fraction of agents with tasks this tick
}
