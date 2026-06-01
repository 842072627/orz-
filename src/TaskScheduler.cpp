// TaskScheduler.cpp — Combined Track: ORZ用strategy_centerline.h, 其他图用默认
// V2: centerline贪心调度取代tidal走廊潮汐
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "config.h"
#include "strategy_centerline.h"
#include "pibt_lock.h"


#include "CompetitionSystem.h"
#include <boost/program_options.hpp>




namespace po = boost::program_options;

extern po::variables_map vm;
extern std::unique_ptr<BaseSystem> system_ptr;
extern void sigint_handler(int);

namespace PIBT_Lock
{
    int g_agent_blocked[PIBT_Lock::MAX_AGENTS] = {0};
    int g_parked_task[PIBT_Lock::MAX_AGENTS];
}

namespace CenterlineAccess {
    const std::vector<int>* g_branch_dist = nullptr;
    const std::vector<int>* g_main_dist = nullptr;
    const std::vector<int>* g_zone_is_main = nullptr;
    const std::vector<int>* g_mp_loc = nullptr;
    const std::vector<int>* g_mp_main_dist = nullptr;
    int g_mp_count = 0;
    int g_n24c = 0;
    int g_n24r = 0;
    float active_ratio = 0.0f;
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    if (env == nullptr || env->rows != 656 || env->cols != 1491)
    {
        std::cout << "TaskScheduler only supports orz map in this build: rows="
                  << (env == nullptr ? -1 : env->rows)
                  << ", cols=" << (env == nullptr ? -1 : env->cols) << std::endl;
        sigint_handler(1);
        return;
    }
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
        return;
              int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
        return;
    }
    if (has_custom_strategy(env))
    {
        std::string strat = get_orz_strategy();
        if (strat == "centerline")
        {
            int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
            strategy_centerline::plan(limit, env, proposed_schedule);
        }
    }
    else
    {
        int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }
}

