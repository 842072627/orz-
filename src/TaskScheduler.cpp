#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "IronZoneScheduler.h"
#include "PerfLogger.h"

static optsvr::IronZoneScheduler g_ironzone_scheduler;
static bool g_config_loaded = false;

void TaskScheduler::initialize(int preprocess_time_limit)
{
    if (!g_config_loaded) {
        optsvr::cfg().load_from_env();
        g_config_loaded = true;
    }

    optsvr::perf().on_init_start();

    // 地图识别 + MapTuner 调参
    optsvr::map_profile() = optsvr::analyze_map(env);
    optsvr::apply_map_tuning(optsvr::map_profile(), env);

    // BFS 预计算（云端64GB可跑，本地用env var关闭）
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

    // IronZoneScheduler 初始化
    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.initialize(env, preprocess_time_limit);
    }

    if (optsvr::cfg().skip_test) {
        optsvr::perf().on_init_end();
        return;
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

    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.schedule(limit, proposed_schedule, env);
    } else {
        DefaultPlanner::schedule_plan(limit, proposed_schedule, env);
    }

    optsvr::perf().on_schedule_done();
}
