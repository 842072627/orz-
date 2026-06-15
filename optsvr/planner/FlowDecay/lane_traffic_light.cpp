#include "lane_traffic_light.h"

namespace IronLane {

int g_lane_lock_dir = 1;  // default: allow comp1→comp2

int g_blocked_12 = 0;
int g_blocked_21 = 0;
int g_passed_12 = 0;
int g_passed_21 = 0;

int* g_exit_to_comp1 = nullptr;
int* g_exit_to_comp2 = nullptr;
bool g_exit_target_initialized = false;

int g_redirected_agents[MAX_AGENTS] = {0};
int g_redirected_count = 0;

int g_prev_lock = 1;

// ── Clearance mode ─────────────────────────────────────────────────────────
bool g_clearance_mode = false;
int  g_clearance_start_tick = -1;
int  g_clearance_to_dir = 0;
int  g_clearance_trigger = 0;

} // namespace IronLane
