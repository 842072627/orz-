#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"

// Best strategy: efficiency-based with region affinity
#include "strategy_efficiency.h"

void TaskScheduler::initialize(int preprocess_time_limit) {
    strategy_init();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    strategy_plan(time_limit, env, proposed_schedule);
}
