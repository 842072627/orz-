// optsvr/schedule/OptRoomSchedule/OptRoomScheduler.h
// 针对 room 地图 3000 tick 定制优化
// 打分公式沿用 DHFULLSchedule r<300 分支，加入 Reclaim + 可调参数
#pragma once

#include "SharedEnv.h"
#include <vector>

namespace orz_engine {

class OptRoomScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env);
};

} // namespace orz_engine
