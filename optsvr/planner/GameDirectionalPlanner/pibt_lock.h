// Compatibility shim for IDEA/gamePlanner's my_planner sources.
#pragma once

#include "config.h"
#include <vector>

namespace PIBT_Lock {
    static constexpr int MAX_AGENTS = 10000;

    inline int g_agent_blocked[MAX_AGENTS] = {0};
    inline int g_parked_task[MAX_AGENTS] = {0};
}

namespace CenterlineAccess {
    inline const std::vector<int>* g_branch_dist = nullptr;
    inline const std::vector<int>* g_main_dist = nullptr;
    inline const std::vector<int>* g_zone_is_main = nullptr;
    inline const std::vector<int>* g_mp_loc = nullptr;
    inline const std::vector<int>* g_mp_main_dist = nullptr;
    inline int g_mp_count = 0;
    inline int g_n24c = 0;
    inline int g_n24r = 0;
    inline float active_ratio = 0.0f;
}
