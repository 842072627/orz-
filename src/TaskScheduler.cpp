// TaskScheduler.cpp — Combined Track: ORZ用strategy_centerline.h, 其他图用默认
// V2: centerline贪心调度取代tidal走廊潮汐
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "config.h"
#include "strategy_centerline.h"

void TaskScheduler::initialize(int preprocess_time_limit)
{
    if (has_custom_strategy(env))
    {
        std::string strat = get_orz_strategy();
        if (strat == "centerline")
        {
            strategy_centerline::init(env);
        }
        else
        {
            // strategy_tidal disabled
        }
        int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    }
    else
    {
        int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    }
}

void TaskScheduler::plan(int time_limit, std::vector<int> &proposed_schedule)
{
    if (has_custom_strategy(env))
    {
        std::string strat = get_orz_strategy();
        if (strat == "centerline")
        {
            int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
            strategy_centerline::plan(limit, env, proposed_schedule);
        }
        else
        {
            // strategy_tidal disabled
        }
    }
    else
    {
        int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }
}
