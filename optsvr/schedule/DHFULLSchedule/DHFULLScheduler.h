// optsvr/schedule/DHFULLSchedule/DHFULLScheduler.h
// 完全拷贝 IDEA/DHFULL 的调度逻辑，只做接口对接
#pragma once

#include "SharedEnv.h"
#include <vector>

namespace optsvr {

class DHFULLScheduler {
public:
    void initialize(SharedEnvironment* env, int preprocess_time_limit_ms);
    void schedule(int time_limit_ms, std::vector<int>& proposed_schedule, SharedEnvironment* env);
};

} // namespace optsvr
