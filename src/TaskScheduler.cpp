#include "TaskScheduler.h"
#include "RuntimeConfig.h"
#include "GameBalanceOptScheduler.h"

optsvr::GameBalanceOptScheduler g_gamebalance_scheduler;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    g_gamebalance_scheduler.initialize(env, preprocess_time_limit);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    g_gamebalance_scheduler.schedule(time_limit, proposed_schedule, env);
}
