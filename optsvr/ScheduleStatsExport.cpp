#include "ScheduleStatsExport.h"

namespace optsvr {
namespace {
GamePrefilterStats game_stats;
}

nlohmann::json get_schedule_stats_json() { return nullptr; }
void set_schedule_result_summary(int, int) {}
void record_schedule_task_progress(int, int) {}
void set_game_prefilter_stats(const GamePrefilterStats& stats) {
    game_stats = stats;
}
GamePrefilterStats get_game_prefilter_stats() { return game_stats; }

}  // namespace optsvr
