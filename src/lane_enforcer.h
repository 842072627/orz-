// lane_enforcer.h - Lane direction enforcement for PIBT
// Agents can only move into lane cells in their permitted direction.
#pragma once
#include "lanes_data.h"
#include "config.h"

namespace LaneEnforcer {

// Check if agent can move to a target cell given its travel direction
// travel_dir: +1 (increasing main_dist) or -1 (decreasing main_dist)
// Returns: true if movement is allowed, false if blocked by lane rules
inline bool can_enter(int target_loc, int travel_dir) {
    if (!travel_dir) return true;  // no direction info, allow
    int8_t lane_dir = LaneData::get_lane_dir(target_loc);
    if (lane_dir == 0) return true;  // no lane restriction
    return lane_dir == travel_dir;   // must match lane direction
}

} // namespace LaneEnforcer
