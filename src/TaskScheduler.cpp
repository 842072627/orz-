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
#include <iostream>

nz::GameBalanceOptScheduler g_gamebalance_scheduler;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    nz::perf().on_init_start();

    nz::load_map_tuning(env);

    if (nz::cfg().skip_test) {
        nz::perf().on_init_end();
        return;
    }

    if (nz::cfg().precompute_heuristic && !nz::cfg().skip_test) {
        auto now = std::chrono::steady_clock::now();
        auto deadline = env->plan_start_time
                      + std::chrono::milliseconds(preprocess_time_limit)
                      - std::chrono::milliseconds(500);
        if (deadline > now + std::chrono::milliseconds(100)) {
            nz::heuristic_matrix().build(env, nz::cfg().num_threads, deadline,
                                             nz::cfg().turn_aware_heuristic);
        }
    }

    if (nz::cfg().schedule_mode == 8) {
        g_gamebalance_scheduler.initialize(env, preprocess_time_limit);
    }

    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);

    nz::perf().on_init_end();
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    if (nz::cfg().skip_test) return;

    nz::perf().on_round_begin();

    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;

    if (nz::cfg().schedule_mode == 8) {
        g_gamebalance_scheduler.schedule(limit, proposed_schedule, env);
    } else {
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }

    nz::perf().on_schedule_done();
}
