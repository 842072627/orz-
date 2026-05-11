// TaskScheduler.cpp — Combined Track: ORZ用strategy_centerline.h, 其他图用默认
// V2: centerline贪心调度取代tidal走廊潮汐
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "config.h"
#include "strategy_centerline.h"
#include "pibt_lock.h"
// #include "CompetitionSystem.h"
// #include <boost/program_options.hpp>
// #include <iostream>

// namespace po = boost::program_options;
// extern po::variables_map vm;
// extern std::unique_ptr<BaseSystem> system_ptr;
// #include <cstdlib>

namespace PIBT_Lock
{
    int g_seg_lock_dir[256] = {0};
    int g_agent_blocked[2000] = {0};
    int g_parked_task[2000];
}

void TaskScheduler::initialize(int preprocess_time_limit)
{

    if (!is_ORZ(env))
    {
        int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    };
    // if (env == nullptr || env->rows != 656 || env->cols != 1491)
    // {
    //     std::cout << "TaskScheduler only supports orz map in this build: rows="
    //               << (env == nullptr ? -1 : env->rows)
    //               << ",cols=" << (env == nullptr ? -1 : env->cols) << std::endl;
    //     exit(1);
    //     return;
    // }

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

    if (!is_ORZ(env)){
              int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }
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
