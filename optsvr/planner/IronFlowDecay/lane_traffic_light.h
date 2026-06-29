#pragma once
// Shared state for iron lane traffic light.
// PIBT checks this before allowing an agent to step into a lane cell.

namespace IronLane {

constexpr int MAX_AGENTS = 10000;

// Lock direction: +1 = allow comp1->comp2, -1 = allow comp2->comp1, 0 = clearance.
extern int g_lane_lock_dir;

// Flow stats for current vote period.
extern int g_blocked_12;
extern int g_blocked_21;
extern int g_passed_12;
extern int g_passed_21;

// Exit targets: nearest non-lane walkable cell on each side for every map cell.
extern int* g_exit_to_comp1;
extern int* g_exit_to_comp2;
extern bool g_exit_target_initialized;

extern int g_redirected_agents[MAX_AGENTS];
extern int g_redirected_count;
extern int g_exempt_from_traffic_light[MAX_AGENTS];
extern int g_prev_lock;

extern bool g_clearance_mode;
extern int  g_clearance_start_tick;
extern int  g_clearance_to_dir;
extern int  g_clearance_trigger;

} // namespace IronLane
