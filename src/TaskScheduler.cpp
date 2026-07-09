#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "GameBalanceOptScheduler.h"
#include "PerfLogger.h"

#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>

optsvr::GameBalanceOptScheduler g_gamebalance_scheduler;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    optsvr::perf().on_init_start();

    optsvr::load_map_tuning(env);

    if (optsvr::cfg().skip_test) {
        optsvr::perf().on_init_end();
        return;
    }

    if (optsvr::cfg().precompute_heuristic && !optsvr::cfg().skip_test) {
        auto now = std::chrono::steady_clock::now();
        auto deadline = env->plan_start_time
                      + std::chrono::milliseconds(preprocess_time_limit)
                      - std::chrono::milliseconds(500);
        if (deadline > now + std::chrono::milliseconds(100)) {
            optsvr::heuristic_matrix().build(env, optsvr::cfg().num_threads, deadline,
                                             optsvr::cfg().turn_aware_heuristic);
        }
    }

    if (optsvr::cfg().schedule_mode == 8) {
        g_gamebalance_scheduler.initialize(env, preprocess_time_limit);
    }

    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);

    optsvr::perf().on_init_end();
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    if (optsvr::cfg().skip_test) return;

    optsvr::perf().on_round_begin();

    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;

    if (optsvr::cfg().schedule_mode == 8) {
        g_gamebalance_scheduler.schedule(limit, proposed_schedule, env);
    } else {
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }

    optsvr::perf().on_schedule_done();
}
