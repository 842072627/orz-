// Lane traffic-light data shim from the current IDEA/IRON snapshot.
// The historical dense lane table is not present in this workspace.
#pragma once

namespace IronLane {

constexpr int MAP_W = 1800;
constexpr int MAP_H = 1912;
constexpr int MAP_SIZE = 3441600;
constexpr int GUARD_DIST = 20;

inline int get_component(int) { return 0; }
inline bool is_core_cell(int) { return false; }
inline bool is_lane_cell(int) { return false; }
inline int manhattan_to_lane(int, int) { return 99; }

} // namespace IronLane
