#pragma once
// Shared state between scheduler (Centerline) and PIBT for direction ahead-check

#include "../src/config.h"

namespace PIBT_Lock {
    static constexpr int MAX_SEGMENTS = 256;
    static constexpr int MAX_AGENTS = 2000;

    extern int g_seg_lock_dir[MAX_SEGMENTS];  // lock direction per segment
    extern int g_agent_blocked[MAX_AGENTS];    // 1 = PIBT should wait, 0 = normal
}
