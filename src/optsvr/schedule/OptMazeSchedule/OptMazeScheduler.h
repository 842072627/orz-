// optsvr/schedule/OptMazeSchedule/OptMazeScheduler.h
// 针对 maze 地图 1500 tick 短时赛道的定制优化
// 打分公式沿用 DHFULLSchedule r<300 分支（已验证最优），加入 Reclaim 机制
#pragma once

#include "SharedEnv.h"
#include <vector>

namespace optsvr {

class OptMazeScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env);
};

} // namespace optsvr
