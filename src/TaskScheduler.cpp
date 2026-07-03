#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "RuntimeConfig.h"
#include "MapProfile.h"
#include "MapTuner.h"
#include "HeuristicMatrix.h"
#include "RegionLoadModel.h"
#include "OptScheduler.h"
#include "IronZoneScheduler.h"
#include "PerfLogger.h"

#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <unordered_set>

optsvr::OptScheduler g_opt_scheduler;
optsvr::IronZoneScheduler g_ironzone_scheduler;

// 自适应策略：tick 0 根据任务分布决定用哪个 scheduler/planner
int g_active_scheduler_mode = -1;  // -1=未决定, 1=堆分配(A), 10=IronZone(B)

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

// 统计已揭示任务的 zone 覆盖数
int count_task_zones(SharedEnvironment* env) {
    if (!env || env->cols <= 0) return 0;
    int zone_rows = 40, zone_cols = 20;
    int cell_h = std::max(1, env->rows / zone_rows);
    int cell_w = std::max(1, env->cols / zone_cols);
    std::unordered_set<int> zones;
    for (const auto& [tid, task] : env->task_pool) {
        if (task.locations.empty()) continue;
        int loc = task.locations[0];
        int r = loc / env->cols, c = loc % env->cols;
        int zr = r / cell_h, zc = c / cell_w;
        zones.insert(zr * zone_cols + zc);
    }
    return static_cast<int>(zones.size());
}

}  // namespace

void TaskScheduler::initialize(int preprocess_time_limit)
{
    optsvr::perf().on_init_start();

    optsvr::load_map_tuning(env);

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

    // 两个 scheduler 都初始化
    g_opt_scheduler.initialize(env, preprocess_time_limit);

    // IronZone 初始化前临时设参数（副本7策略B）
    auto& c = optsvr::cfg();
    float saved_lock = c.iron_future_flow_lock_th;
    float saved_cap = c.iron_future_flow_congest_cap;
    int saved_zr = c.iron_future_flow_zone_rows;
    int saved_zc = c.iron_future_flow_zone_cols;
    bool saved_zt = c.iron_ff_zone_tuning_enabled;
    float saved_lc = c.iron_ff_left_edge_congest_cap;
    float saved_rc = c.iron_ff_right_edge_congest_cap;
    float saved_tc = c.iron_ff_top_congest_cap;
    float saved_bc = c.iron_ff_bottom_congest_cap;
    float saved_lcp = c.iron_ff_left_corner_penalty;
    float saved_rcp = c.iron_ff_right_corner_penalty;
    float saved_op = c.iron_ff_outer_penalty;
    float saved_lep = c.iron_ff_left_edge_penalty;
    float saved_dist = c.iron_dist_weight;

    c.iron_future_flow_lock_th = 9999.0f;
    c.iron_future_flow_congest_cap = 9999.0f;
    c.iron_future_flow_zone_rows = 20;
    c.iron_future_flow_zone_cols = 50;
    c.iron_ff_zone_tuning_enabled = true;
    c.iron_ff_left_edge_congest_cap = 45.0f;
    c.iron_ff_right_edge_congest_cap = 200.0f;
    c.iron_ff_top_congest_cap = 200.0f;
    c.iron_ff_bottom_congest_cap = 200.0f;
    c.iron_ff_left_corner_penalty = 40.0f;
    c.iron_ff_right_corner_penalty = 180.0f;
    c.iron_ff_outer_penalty = 30.0f;
    c.iron_ff_left_edge_penalty = 100.0f;
    c.iron_dist_weight = 9.0f;

    g_ironzone_scheduler.initialize(env, preprocess_time_limit);

    // 恢复堆分配参数（IronZone 已用策略B参数初始化完毕，保留策略B参数）
    // 如果 tick 0 选策略A，IronZone 不会被调用，参数无所谓
    // 如果 tick 0 选策略B，参数已经正确

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

    // tick 0: 根据任务分布决定策略
    if (g_active_scheduler_mode < 0) {
        int zones = count_task_zones(env);
        int agents = env ? env->num_of_agents : 0;
        if (zones > 0 && zones < 150 && agents > 7000) {
            g_active_scheduler_mode = 10;  // IronZone 策略B
        } else {
            g_active_scheduler_mode = 1;   // 堆分配 策略A
        }
        std::cerr << "[Adaptive] tick=0 zones=" << zones << " agents=" << agents
                  << " -> mode=" << g_active_scheduler_mode << "\n";
    }

    switch (g_active_scheduler_mode) {
        case 10: g_ironzone_scheduler.schedule(limit, proposed_schedule, env); break;
        case 1:  g_opt_scheduler.schedule(limit, proposed_schedule, env); break;
        default: DefaultPlanner::schedule_plan(limit, proposed_schedule, env); break;
    }

    if (optsvr::cfg().timing_trace) {
        const auto schedule_end = std::chrono::steady_clock::now();
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            schedule_end - schedule_start).count();
        std::cerr << "[TIMING][SCHEDULE]"
                  << " mode=" << g_active_scheduler_mode
                  << " elapsed=[" << elapsed_ms << "ms]"
                  << " limit=" << limit << "ms"
                  << " entry_limit=" << time_limit << "ms"
                  << "\n";
    }

    optsvr::perf().on_schedule_done();
}
