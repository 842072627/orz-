#include "TaskScheduler.h"
#include "GameBalanceOptScheduler.h"
#include "HeuristicMatrix.h"
#include "MapTuner.h"
#include "RuntimeConfig.h"
#include "const.h"
#include "scheduler.h"

#include <algorithm>
#include <chrono>

namespace {
optsvr::GameBalanceOptScheduler game_scheduler;
}

void TaskScheduler::initialize(int preprocess_time_limit) {
    optsvr::load_map_tuning(env);
    if (optsvr::cfg().skip_test) return;

    const auto now = std::chrono::steady_clock::now();
    const auto deadline = env->plan_start_time
        + std::chrono::milliseconds(preprocess_time_limit)
        - std::chrono::milliseconds(500);
    if (deadline > now + std::chrono::milliseconds(100)) {
        optsvr::heuristic_matrix().build(env, optsvr::cfg().num_threads,
                                         deadline, false);
    }
    game_scheduler.initialize(env, preprocess_time_limit);
    DefaultPlanner::schedule_initialize(
        preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE,
        env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    if (optsvr::cfg().skip_test) return;
    game_scheduler.schedule(
        time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE,
        proposed_schedule, env);
}
