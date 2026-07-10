// nz/ScheduleStatsExport.h — 调度诊断指标导出接口
// 由 OptRoomScheduler.cpp 实现，saveResults() 调用
#pragma once

#include "nlohmann/json.hpp"
#include <string>

namespace nz {

// 获取调度统计 JSON（由当前活跃的 Scheduler 实现）
// 如果没有数据返回 nullptr
nlohmann::json get_schedule_stats_json();

void set_schedule_result_summary(int num_task_finished, int final_tick);
void record_schedule_task_progress(int timestep, int cumulative_finished);

struct GamePrefilterStats {
    int total = 0;
    int kept = 0;
    int rejected = 0;
    int missing = 0;
    int channel = 0;
    int capacity = 0;
    int flow = 0;
    int lock = 0;
    int density = 0;
    int branch = 0;
    int other = 0;
    int blocked_zones = 0;
    std::string blocked_zone_ids;
};

void set_game_prefilter_stats(const GamePrefilterStats& stats);
GamePrefilterStats get_game_prefilter_stats();

} // namespace nz
