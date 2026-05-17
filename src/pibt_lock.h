#pragma once
// Shared state between scheduler (Centerline) and PIBT for direction ahead-check

#include "../src/config.h"

namespace PIBT_Lock {
    static constexpr int MAX_SEGMENTS = 1024;
    static constexpr int MAX_AGENTS = 10000;

    extern int g_seg_lock_dir[MAX_SEGMENTS];  // lock direction per segment
    extern int g_agent_blocked[MAX_AGENTS];    // 1 = PIBT should wait, 0 = normal
    extern int g_parked_task[MAX_AGENTS];      // pending second errand task id, -1 = none

    // Cold zone pushout shared state (dummy goal override)
    extern int g_cold_goals[MAX_AGENTS];        // cold zone location, -1=none
    extern int g_agent_bound_lock[MAX_AGENTS];  // bound lock square index, -1=none
}
