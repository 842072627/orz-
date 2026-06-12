#pragma once
// Shared state for iron lane traffic light.
// PIBT checks this before allowing an agent to step into a lane cell.

namespace IronLane {

// Max agents
constexpr int MAX_AGENTS = 10000;

// Lock direction: +1 = allow comp1→comp2, -1 = allow comp2→comp1, never 0
extern int g_lane_lock_dir;

// Flow stats for current period (accumulated, reset every VOTE_PERIOD ticks)
extern int g_blocked_12;  // comp1→comp2 attempted entry blocked
extern int g_blocked_21;  // comp2→comp1 attempted entry blocked
extern int g_passed_12;   // comp1→comp2 entry allowed
extern int g_passed_21;   // comp2→comp1 entry allowed

// Exit targets: for each map cell, nearest non-lane walkable cell on each side.
// exit_to_comp1[loc] = nearest exit on comp1 side (for agents needing to retreat to comp1)
// exit_to_comp2[loc] = nearest exit on comp2 side (for agents needing to retreat to comp2)
extern int* g_exit_to_comp1;
extern int* g_exit_to_comp2;
extern bool g_exit_target_initialized;

// Redirected agent tracking (agent in lane going counterflow)
extern int g_redirected_agents[MAX_AGENTS];
extern int g_redirected_count;

// Previous lock for detecting flips
extern int g_prev_lock;

} // namespace IronLane
