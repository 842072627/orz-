// optsvr/ScheduleStatsExport.h — 调度诊断指标导出接口
// 由当前实验 scheduler 提供数据，saveResults() 调用
#pragma once

#include "nlohmann/json.hpp"

namespace optsvr {

// 获取调度统计 JSON（由当前活跃的 Scheduler 实现）
// 如果没有数据返回 nullptr
nlohmann::json get_schedule_stats_json();
nlohmann::json get_optroom_schedule_stats_json();
nlohmann::json get_game_segment_flow_stats_json();
void set_schedule_result_summary(int num_task_finished, int final_tick);
void set_game_segment_flow_result_summary(int num_task_finished, int final_tick);
void record_schedule_task_progress(int timestep, int cumulative_finished);
void record_game_segment_flow_task_progress(int timestep, int cumulative_finished);

} // namespace optsvr
