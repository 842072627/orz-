#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"

// Strategy selection: switch by editing this include
// Available: "strategy_efficiency.h", "strategy_task_urgency.h"
#include "strategy_swap_opt.h"

void TaskScheduler::initialize(int preprocess_time_limit) {
    strategy_swap_opt::init();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    strategy_swap_opt::plan(time_limit, env, proposed_schedule);
}
