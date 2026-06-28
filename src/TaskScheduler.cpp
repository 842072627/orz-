#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "RegionLoadModel.h"
#include "OptScheduler.h"
#include "DHScheduler.h"
#include "GameOptScheduler.h"
#include "GameBalanceOptScheduler.h"
#include "DHFULLScheduler.h"
#include "MazeOptScheduler.h"
#include "OptMazeScheduler.h"
#include "OptRoomScheduler.h"
#include "IronZoneScheduler.h"
#include "RandomRouteLoadScheduler.h"
#include "PerfLogger.h"

#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>

optsvr::OptScheduler g_opt_scheduler;
optsvr::DHScheduler  g_dh_scheduler;
optsvr::GameOptScheduler g_game_scheduler;
optsvr::GameBalanceOptScheduler g_gamebalance_scheduler;
optsvr::DHFULLScheduler  g_dhfull_scheduler;
optsvr::MazeOptScheduler g_mazeopt_scheduler;
optsvr::OptMazeScheduler g_optmaze_scheduler;
optsvr::OptRoomScheduler g_optroom_scheduler;
optsvr::IronZoneScheduler g_ironzone_scheduler;
optsvr::RandomRouteLoadScheduler g_random_route_load_scheduler;

namespace {

std::string region_export_path(const SharedEnvironment* env,
                               const optsvr::RegionLoadConfig& cfg) {
    std::string stem = env && !env->map_name.empty() ? env->map_name : "map";
    size_t dot = stem.find_last_of('.');
    if (dot != std::string::npos) stem = stem.substr(0, dot);
    for (char& ch : stem) {
        unsigned char u = static_cast<unsigned char>(ch);
        if (!std::isalnum(u) && ch != '-' && ch != '_') ch = '_';
    }
    std::filesystem::path out_dir(cfg.export_dir);
    return (out_dir / (stem + "_regions.json")).string();
}

}  // namespace

void TaskScheduler::initialize(int preprocess_time_limit)
{
    optsvr::perf().on_init_start();

    // 地图识别 + MapTuner 调参（设置 skip_test 等），必须在 scheduler 初始化之前
    optsvr::load_map_tuning(env);

    // Optional region abstraction for future task scoring. Disabled by default.
    auto region_cfg = optsvr::RegionLoadConfig::from_env();
    if (region_cfg.enabled) {
        auto& model = optsvr::region_load_model();
        model.initialize(env, region_cfg);
        if (model.ready()) {
            std::string out_path = region_export_path(env, region_cfg);
            if (model.export_json(out_path)) {
                std::cerr << "[RegionLoadModel] mode=" << region_cfg.mode_name()
                          << " regions=" << model.num_regions()
                          << " free=" << model.n_free()
                          << " export=" << out_path << "\n";
            }
        } else {
            std::cerr << "[RegionLoadModel] initialization skipped or failed\n";
        }
    }

    // ---- 距离矩阵预计算（共用，对所有方案生效；mode=5/7/9 有独立初始化，跳过） ----
    if (optsvr::cfg().precompute_heuristic && !optsvr::cfg().skip_test
        && optsvr::cfg().schedule_mode != 5 && optsvr::cfg().schedule_mode != 7
        && optsvr::cfg().schedule_mode != 9) {
        auto now = std::chrono::steady_clock::now();
        auto deadline = env->plan_start_time
                      + std::chrono::milliseconds(preprocess_time_limit)
                      - std::chrono::milliseconds(500);
        if (deadline > now + std::chrono::milliseconds(100)) {
            optsvr::heuristic_matrix().build(env, optsvr::cfg().num_threads, deadline,
                                             optsvr::cfg().turn_aware_heuristic);
        }
    }

    if (optsvr::cfg().schedule_mode == 1 || optsvr::cfg().schedule_mode == 4) {
        g_opt_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 2) {
        g_dh_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 3) {
        g_game_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 8) {
        g_gamebalance_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 5) {
        g_dhfull_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 9) {
        g_mazeopt_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 7) {
        g_optroom_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 10) {
        g_ironzone_scheduler.initialize(env, preprocess_time_limit);
    }
    if (optsvr::cfg().schedule_mode == 11) {
        g_random_route_load_scheduler.initialize(env, preprocess_time_limit);
    }

    if (optsvr::cfg().skip_test) {
        optsvr::perf().on_init_end();
        return;
    }

    // mode=5/7/9 内部已自行调用 DefaultPlanner::schedule_initialize
    if (optsvr::cfg().schedule_mode != 5 && optsvr::cfg().schedule_mode != 7
        && optsvr::cfg().schedule_mode != 9) {
        int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
        DefaultPlanner::schedule_initialize(limit, env);
    }

    optsvr::perf().on_init_end();
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    if (optsvr::cfg().skip_test) return;

    optsvr::perf().on_round_begin();

    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    const auto schedule_start = std::chrono::steady_clock::now();

    switch (optsvr::cfg().schedule_mode) {
        case 9:  g_mazeopt_scheduler.schedule(limit, proposed_schedule, env); break;
        case 8:  g_gamebalance_scheduler.schedule(limit, proposed_schedule, env); break;
        case 7:  g_optroom_scheduler.schedule(limit, proposed_schedule, env); break;
        case 6:  g_optmaze_scheduler.schedule(limit, proposed_schedule, env); break;
        case 5:  g_dhfull_scheduler.schedule(limit, proposed_schedule, env); break;
        case 10: g_ironzone_scheduler.schedule(limit, proposed_schedule, env); break;
        case 11: g_random_route_load_scheduler.schedule(limit, proposed_schedule, env); break;
        case 3:  g_game_scheduler.schedule(limit, proposed_schedule, env); break;
        case 2:  g_dh_scheduler.schedule(limit, proposed_schedule, env); break;
        case 4:  // TaskMatch（与 GreedyHeap 共用 OptScheduler，内部按 schedule_mode 分流）
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

    optsvr::perf().on_schedule_done();
}
