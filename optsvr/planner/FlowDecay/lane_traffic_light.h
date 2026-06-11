#pragma once
// Shared state between scheduler and FlowDecay PIBT for lane traffic light.
// The PIBT checks this before allowing an agent to step into a lane cell.

namespace IronLane {

// Set by scheduler each tick based on agents currently in core cells
// +1 = agents from comp1 are in core (lock direction comp1→comp2)
// -1 = agents from comp2 are in core (lock direction comp2→comp1)
//  0 = no agents in core, free passage
extern int g_lane_lock_dir;

} // namespace IronLane
