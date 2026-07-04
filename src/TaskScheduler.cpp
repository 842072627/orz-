#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "IronZoneScheduler.h"
#include "PerfLogger.h"

#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>

optsvr::IronZoneScheduler g_ironzone_scheduler;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    optsvr::perf().on_init_start();

    // 地图识别 + MapTuner 调参（设置 skip_test 等），必须在 scheduler 初始化之前
    optsvr::load_map_tuning(env);

    // ---- 距离矩阵预计算（共用） ----
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

    // IronZone scheduler (schedule_mode=10)
    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.initialize(env, preprocess_time_limit);
    }

    if (optsvr::cfg().skip_test) {
        optsvr::perf().on_init_end();
        return;
    }

    DefaultPlanner::schedule_initialize(preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE, env);

    optsvr::perf().on_init_end();
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    if (optsvr::cfg().skip_test) return;

    optsvr::perf().on_round_begin();

    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    const auto schedule_start = std::chrono::steady_clock::now();

    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.schedule(limit, proposed_schedule, env);
    } else {
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }

    if (optsvr::cfg().timing_trace) {
        const auto schedule_end = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            schedule_end - schedule_start).count();
        std::cerr << "[TIMING][SCHEDULE]"
                  << " mode=" << optsvr::cfg().schedule_mode
                  << " elapsed=[" << elapsed_ms << "ms]"
                  << " limit=" << limit << "ms"
                  << " entry_limit=" << time_limit << "ms"
                  << "\n";
    }

    optsvr::perf().on_schedule_done();
}

void TaskScheduler::idle_relocate(std::vector<int> & proposed_schedule)
{
    if (optsvr::cfg().skip_test) return;
    if (optsvr::cfg().schedule_mode != 10) return;

    g_ironzone_scheduler.idle_relocate(proposed_schedule, env);
}
