#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"


#include "OptScheduler.h"


#include "IronZoneScheduler.h"


#include <chrono>
#include <iostream>

optsvr::OptScheduler g_opt_scheduler;
optsvr::IronZoneScheduler g_ironzone_scheduler;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    // 地图识别 + MapTuner 调参（设置 skip_test 等），必须在 scheduler 初始化之前
    optsvr::load_map_tuning(env);

    if (optsvr::cfg().skip_test) {
        return;
    }

    if (optsvr::cfg().schedule_mode == 1 || optsvr::cfg().schedule_mode == 4) {
        g_opt_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.initialize(env, preprocess_time_limit);
    }

    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    if (optsvr::cfg().skip_test) return;

    

    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    const auto schedule_start = std::chrono::steady_clock::now();

    switch (optsvr::cfg().schedule_mode) {
        case 10: g_ironzone_scheduler.schedule(limit, proposed_schedule, env); break;
        case 4:  // TaskMatch（与 GreedyHeap 共用 OptScheduler）
        case 1:  g_opt_scheduler.schedule(limit, proposed_schedule, env); break;
        default: DefaultPlanner::schedule_plan(limit, proposed_schedule, env); break;
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

    
}
