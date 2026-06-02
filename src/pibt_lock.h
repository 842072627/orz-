#pragma once
// Shared state between scheduler (Centerline) and PIBT for direction ahead-check

#include "../src/config.h"
#include <vector>

namespace PIBT_Lock {
    static constexpr int MAX_AGENTS = 10000;

    extern int g_agent_blocked[MAX_AGENTS];    // 1 = PIBT should wait, 0 = normal
    extern int g_parked_task[MAX_AGENTS];      // pending second errand task id, -1 = none
    extern int g_agent_refuge[MAX_AGENTS];     // 1 = agent in refuge (excluded from density/flow stats)
    extern int g_agent_refuge_goal[MAX_AGENTS]; // refuge target grid cell, -1 = none
    extern int g_agent_refuge_lane[MAX_AGENTS]; // which lane agent is bound to, -1 = none
    extern int g_agent_refuge_lock_dir[MAX_AGENTS]; // lock_dir at binding time, 0 = none
}

// Centerline data shared with PIBT for wall-hugging (set by scheduler after init)
namespace CenterlineAccess {
    extern const std::vector<int>* g_branch_dist;     // [loc]
    extern const std::vector<int>* g_main_dist;       // [loc]
    extern const std::vector<int>* g_zone_is_main;    // [zone_id]
    extern const std::vector<int>* g_mp_loc;          // [si] = main point map location
    extern const std::vector<int>* g_mp_main_dist;    // [si] = main_dist at main point
    extern const std::vector<int>* g_mp_width_p10;    // [si] = P10 corridor width
    extern const std::vector<int>* g_mp_lock_dir;     // [si] = direction lock (+1/-1/0)
    extern const std::vector<int>* g_zone24_density;  // [zid] = live agent count per 24x24 zone
    extern int g_mp_count;
    extern int g_n24c;
    extern int g_n24r;
    extern float active_ratio;       // [0-1] fraction of agents with tasks this tick
}
