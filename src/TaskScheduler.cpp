// TaskScheduler.cpp — Combined Track: ORZ用V7调度器，其他图用默认
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "config.h"
#include "strategy_heat.h"

void TaskScheduler::initialize(int preprocess_time_limit) {
    if (has_custom_strategy(env)) {
        // ORZ: 自定义V7调度器
        strategy_heat::init();
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    } else {
        // 其他图: 默认调度器
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    }
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    if (has_custom_strategy(env)) {
        // ORZ: V7调度
        strategy_heat::plan(time_limit, env, proposed_schedule);
    } else {
        // 其他图: 默认调度
        int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }
}
