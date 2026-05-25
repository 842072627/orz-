#pragma once
// Shared state between scheduler (Centerline) and PIBT for direction ahead-check

#include "../src/config.h"
#include <vector>

namespace PIBT_Lock {
    static constexpr int MAX_AGENTS = 10000;

    extern int g_agent_blocked[MAX_AGENTS];    // 1 = PIBT should wait, 0 = normal
    extern int g_parked_task[MAX_AGENTS];      // pending second errand task id, -1 = none
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
#if USE_TRAFFIC_PREDICTION
    extern const void* g_segments_ptr;  // points to std::vector<Centerline::TrafficSegment>*
#endif
}
