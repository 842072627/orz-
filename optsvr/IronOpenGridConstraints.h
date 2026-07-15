#pragma once

#include "SharedEnv.h"
#include "RuntimeConfig.h"

#include <cstdint>

namespace optsvr {

// Direction encoding: 0=E, 1=S, 2=W, 3=N.
// Forbid-mask encoding follows IronHighway: W=1, E=2, N=4, S=8.
inline uint8_t iron_open_grid_forbid_mask(
    const SharedEnvironment* env, int location)
{
    if (env == nullptr || !cfg().iron_open_grid_enabled ||
        location < 0 || location >= static_cast<int>(env->map.size()) ||
        env->map[location] != 0 || env->cols <= 0)
        return 0;

    const int row = location / env->cols;
    const int col = location % env->cols;
    const int boundary = cfg().iron_open_grid_boundary_col;
    const int clearance = cfg().iron_open_grid_clearance;
    if (row < clearance || row + clearance >= env->rows ||
        col < clearance || col + clearance >= env->cols ||
        col >= boundary)
        return 0;

    // Keep an unconstrained clearance band around obstacles and region edges.
    // This is the reachability escape network for irregular Iron geometry.
    for (int k = 1; k <= clearance; ++k) {
        if (env->map[location - k] != 0 ||
            env->map[location + k] != 0 ||
            env->map[location - k * env->cols] != 0 ||
            env->map[location + k * env->cols] != 0)
            return 0;
    }

    uint8_t mask = 0;
    const bool split_axes = cfg().iron_open_grid_exclusive_axis &&
        cfg().iron_open_grid_rows_enabled &&
        cfg().iron_open_grid_cols_enabled;
    const int axis_parity = (row + col + cfg().iron_open_grid_axis_phase) & 1;
    const bool use_row = cfg().iron_open_grid_rows_enabled &&
        (!split_axes || axis_parity == 0);
    const bool use_col = cfg().iron_open_grid_cols_enabled &&
        (!split_axes || axis_parity != 0);
    if (use_row)
        mask |= (row % 2 == 0) ? 1 : 2;  // even row east, odd row west
    if (use_col)
        mask |= (col % 2 == 0) ? 4 : 8;  // even col south, odd col north
    return mask;
}

inline bool iron_forbid_mask_blocks_direction(uint8_t mask, int direction)
{
    static constexpr uint8_t bit[4] = {2, 8, 1, 4};
    return direction < 0 || direction >= 4 || (mask & bit[direction]) != 0;
}

} // namespace optsvr
