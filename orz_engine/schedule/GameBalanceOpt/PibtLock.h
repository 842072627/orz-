// optsvr/schedule/GameBalanceOpt/PibtLock.h — Scheduler↔Planner 共享状态
// 方向锁 + agent 阻塞标志 + parked task
#pragma once

#include <cstring>

namespace orz_engine {

struct PibtLock {
    static constexpr int MAX_SEGMENTS = 1024;
    static constexpr int MAX_AGENTS   = 10000;
    static constexpr int MAX_ZONES    = 4096;  // 24x24 zone grid

    int seg_lock_dir[MAX_SEGMENTS];   // zone 方向锁: +1/-1/0
    int agent_blocked[MAX_AGENTS];    // 1=PIBT 应原地等, 0=正常
    int parked_task[MAX_AGENTS];      // 停放的第二 errand task id, -1=无

    // ---- IDEA: gradual priority penalty ----
    float agent_priority_penalty[MAX_AGENTS]; // 0.0=正常, 负值=降低优先级

    // ---- IDEA: zone direction for PIBT counter-flow ----
    int zone_dir[MAX_ZONES];          // zone方向锁: +1/-1/0 (供PIBT查询)
    int zone_n24c;                    // zone grid列数 (供PIBT计算zone id)

    // ---- IDEA10: zone width for flow bottleneck ----
    int zone_width[MAX_ZONES];        // zone最小宽度 (供Planner查询)

    void reset() {
        std::memset(seg_lock_dir, 0, sizeof(seg_lock_dir));
        std::memset(agent_blocked, 0, sizeof(agent_blocked));
        std::memset(parked_task, 0xFF, sizeof(parked_task)); // -1
        std::memset(agent_priority_penalty, 0, sizeof(agent_priority_penalty));
        std::memset(zone_dir, 0, sizeof(zone_dir));
        std::memset(zone_width, 0, sizeof(zone_width));
        zone_n24c = 0;
    }

    static PibtLock& instance() {
        static PibtLock inst;
        return inst;
    }
private:
    PibtLock() { reset(); }
};

inline PibtLock& pibt_lock() { return PibtLock::instance(); }

} // namespace orz_engine
