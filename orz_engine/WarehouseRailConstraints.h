// optsvr/WarehouseRailConstraints.h
// Shared warehouse rail/one-way constraint builder used by mode 13 planner and heuristic BFS.
#pragma once

#include "SharedEnv.h"

#include <algorithm>
#include <array>
#include <vector>

namespace orz_engine {

constexpr int WAREHOUSE_RAIL_FORBID_COST = 1000000;
constexpr int WAREHOUSE_RAIL_GUIDE_PENALTY = 5000;

struct WarehouseRailConstraintStats {
    int passage_rows = 0;
    int passage_cols = 0;
    int forbidden_edges = 0;
};

inline WarehouseRailConstraintStats build_warehouse_planner_rail_constraints(
    SharedEnvironment* env,
    std::vector<std::array<unsigned char, 4>>* forbidden_dirs,
    std::vector<std::vector<int>>* extra_weights)
{
    WarehouseRailConstraintStats stats;
    if (env == nullptr || env->rows <= 2 || env->cols <= 2) return stats;

    const int rows = env->rows;
    const int cols = env->cols;
    const int map_size = rows * cols;
    const int x1 = 7, y1 = 7, x2 = 488, y2 = 131;

    std::vector<std::array<unsigned char, 4>> flags(map_size, {0, 0, 0, 0});
    if (extra_weights != nullptr) {
        extra_weights->assign(map_size, std::vector<int>(4, 0));
    }

    auto inr = [&](int c, int r) {
        return c > x1 && c < x2 && r > y1 && r < y2;
    };

    std::vector<int> row_free(rows, 0);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            if (env->map[r * cols + c] == 0 && inr(c, r)) row_free[r]++;
        }
    }

    int max_rf = *std::max_element(row_free.begin(), row_free.end());
    int row_th = max_rf * 80 / 100;
    std::vector<int> passage_rows;
    for (int r = 1; r < rows - 1; r++) {
        if (row_free[r] > row_th &&
            row_free[r - 1] <= row_th &&
            row_free[r + 1] <= row_th) {
            passage_rows.push_back(r);
        }
    }

    std::vector<int> col_free(cols, 0);
    for (int c = 0; c < cols; c++) {
        for (int r = 0; r < rows; r++) {
            if (env->map[r * cols + c] == 0 && inr(c, r)) col_free[c]++;
        }
    }

    int max_cf = *std::max_element(col_free.begin(), col_free.end());
    int col_th = max_cf * 80 / 100;
    std::vector<int> passage_cols;
    for (int c = 1; c < cols - 1; c++) {
        if (col_free[c] > col_th &&
            col_free[c - 1] <= col_th &&
            col_free[c + 1] <= col_th) {
            passage_cols.push_back(c);
        }
    }

    auto forbid_dir = [&](int loc, int d) {
        if (loc < 0 || loc >= map_size || d < 0 || d >= 4) return;
        if (env->map[loc] != 0) return;
        if (flags[loc][d] == 0) {
            flags[loc][d] = 1;
            stats.forbidden_edges++;
        }
        if (extra_weights != nullptr) {
            (*extra_weights)[loc][d] = WAREHOUSE_RAIL_FORBID_COST;
        }
    };

    const int col_begin = std::max(0, x1 + 1);
    const int col_end = std::min(cols, x2);
    const int row_begin = std::max(0, y1 + 1);
    const int row_end = std::min(rows, y2);

    for (int i = 0; i < static_cast<int>(passage_rows.size()); i++) {
        int row = passage_rows[i];
        int dir = (i % 2 == 0) ? 2 : 0;  // even: forbid W, odd: forbid E
        for (int c = col_begin; c < col_end; c++) {
            forbid_dir(row * cols + c, dir);
        }
    }

    for (int i = 0; i < static_cast<int>(passage_cols.size()); i++) {
        int col = passage_cols[i];
        int dir = (i % 2 == 0) ? 3 : 1;  // even: forbid N, odd: forbid S
        for (int r = row_begin; r < row_end; r++) {
            forbid_dir(r * cols + col, dir);
        }
    }

    for (int loc = 0; loc < map_size; loc++) {
        if (env->map[loc] != 0) continue;
        int r = loc / cols;
        int c = loc % cols;
        if (c == 0 || env->map[loc - 1] == 1) forbid_dir(loc, 2);       // W
        if (c == cols - 1 || env->map[loc + 1] == 1) forbid_dir(loc, 0); // E
        if (r == 0 || env->map[loc - cols] == 1) forbid_dir(loc, 3);     // N
        if (r == rows - 1 || env->map[loc + cols] == 1) forbid_dir(loc, 1); // S
    }

    auto add_guide_penalty = [&](int c, int r, char guide_dir) {
        if (extra_weights == nullptr) return;
        if (r < 0 || r >= rows || c < 0 || c >= cols) return;
        int loc = r * cols + c;
        if (env->map[loc] != 0) return;
        int d = -1;
        if (guide_dir == 'l') d = 0;
        else if (guide_dir == 'r') d = 2;
        else if (guide_dir == 'u') d = 1;
        else if (guide_dir == 'd') d = 3;
        if (d >= 0) (*extra_weights)[loc][d] += WAREHOUSE_RAIL_GUIDE_PENALTY;
    };

    for (int i = 0; i < static_cast<int>(passage_cols.size()); i++) {
        char gdir = (i % 2 == 0) ? 'd' : 'u';
        for (int dc = -1; dc <= 1; dc++) {
            int gc = passage_cols[i] + dc;
            add_guide_penalty(gc, y1, gdir);
            add_guide_penalty(gc, y1 - 1, gdir);
            add_guide_penalty(gc, y2, gdir);
            add_guide_penalty(gc, y2 + 1, gdir);
        }
    }

    for (int i = 0; i < static_cast<int>(passage_rows.size()); i++) {
        char gdir = (i % 2 == 0) ? 'r' : 'l';
        add_guide_penalty(x1, passage_rows[i], gdir);
        add_guide_penalty(x1 - 1, passage_rows[i], gdir);
        add_guide_penalty(x2, passage_rows[i], gdir);
        add_guide_penalty(x2 + 1, passage_rows[i], gdir);
    }

    if (forbidden_dirs != nullptr) {
        forbidden_dirs->swap(flags);
    }

    stats.passage_rows = static_cast<int>(passage_rows.size());
    stats.passage_cols = static_cast<int>(passage_cols.size());
    return stats;
}

} // namespace orz_engine
