// ORZ-only stub
#include "ScheduleStatsExport.h"

namespace nz {
static GamePrefilterStats g_prefilter_stats;
void set_game_prefilter_stats(const GamePrefilterStats& stats) { g_prefilter_stats = stats; }
GamePrefilterStats get_game_prefilter_stats() { return g_prefilter_stats; }
}
