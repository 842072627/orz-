// Auto-generated lane traffic light data for iron map.
#pragma once
#include <cstdlib>

namespace IronLane {

constexpr int MAP_W = 1800;
constexpr int MAP_H = 1912;
constexpr int MAP_SIZE = 3441600;
constexpr int GUARD_DIST = 20;

// O(1) direct-lookup arrays (defined in lane_data.cpp)
extern const signed char g_comp[];      // [MAP_SIZE] 0=wall/lane, 1=comp1, 2=comp2
extern const signed char g_core[];      // [MAP_SIZE] 0=not core, 1=core
extern const signed char g_lane[];      // [MAP_SIZE] 0=not lane, 1=lane

// Manhattan distance to lane (bounding box, 99=outside)
constexpr int MD_OX = 1573;
constexpr int MD_OY = 930;
constexpr int MD_W = 99;
constexpr int MD_H = 166;
extern const signed char g_md[];        // [MD_H * MD_W]

// Fast inline lookups
inline int get_component(int loc) { return (loc >= 0 && loc < MAP_SIZE) ? g_comp[loc] : 0; }
inline bool is_core_cell(int loc) { return (loc >= 0 && loc < MAP_SIZE) && g_core[loc] == 1; }
inline bool is_lane_cell(int loc) { return (loc >= 0 && loc < MAP_SIZE) && g_lane[loc] == 1; }
inline int manhattan_to_lane(int x, int y) {
    int dx = x - MD_OX, dy = y - MD_OY;
    if (dx < 0 || dx >= MD_W || dy < 0 || dy >= MD_H) return 99;
    return g_md[dy * MD_W + dx];
}

} // namespace IronLane
