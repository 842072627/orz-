#include "TaskScheduler.h"
#include "GameBalanceOptScheduler.h"
#include "HeuristicMatrix.h"
#include "MapTuner.h"
#include "RuntimeConfig.h"
#include "const.h"
#include "scheduler.h"

#include <algorithm>
#include <chrono>

#include "CompetitionSystem.h"
#include <boost/program_options.hpp>
#include <iostream>

namespace po = boost::program_options;
extern po::variables_map vm;
extern std::unique_ptr<BaseSystem> system_ptr;
extern void sigint_handler(int);

namespace {
optsvr::GameBalanceOptScheduler game_scheduler;
}

void TaskScheduler::initialize(int preprocess_time_limit) {
    // 非 ORZ 地图直接 skip（使用 sigint_handler 优雅退出）
    if (env == nullptr || env->map_name.find("orz") == std::string::npos) {
        std::cerr << "[TaskScheduler] non-ORZ map, skipping (sigint_handler)\n";
        sigint_handler(1);
        return;
    }
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
