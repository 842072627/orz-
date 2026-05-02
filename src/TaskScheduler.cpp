#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"

// Strategy selection: switch by editing this include
// Available: "strategy_efficiency.h", "strategy_task_urgency.h"
#include "strategy_task_urgency.h"

void TaskScheduler::initialize(int preprocess_time_limit) {
    strategy_task_urgency::init();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    strategy_task_urgency::plan(time_limit, env, proposed_schedule);
}
