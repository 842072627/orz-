#pragma once

#include <cstdint>

namespace PIBT_Lock {
    static constexpr int MAX_AGENTS = 10000;
    inline int g_agent_blocked[MAX_AGENTS] = {0};
    inline int g_pibt_max_depth = 0;          // 每 tick 最大 PIBT 链深度
    inline int g_pibt_total_chains = 0;        // 每 tick 总链数（higher_id==-1 的调用）
    inline int64_t g_pibt_total_depth_sum = 0; // 每 tick 深度总和，用于算平均
} // namespace PIBT_Lock

inline int g_parked_task[PIBT_Lock::MAX_AGENTS] = {0};
