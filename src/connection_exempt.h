// connection_exempt.h
// 连接豁免系统 — agent 从热区逃逸到安全区的最后通道
//
// 一条规则:
//   agent 当前 main zone 热度 < 35 → 尝试逃逸
//     对 task 的每个 segment (2errand) 做 Bresenham 路径检查:
//       ✅ 不穿墙
//       ✅ 不经过 main zone 热度 ≥ 35
//       ✅ 不经过非 main zone 热度 ≥ 80
//     全部通过 → 放行，按直线距离给分

#ifndef CONNECTION_EXEMPT_H
#define CONNECTION_EXEMPT_H

#include <vector>
#include <cmath>
#include <algorithm>

// ====== 配置 ======
constexpr int EX_MAIN_CAP   = 15;  // main zone 阈值
constexpr int EX_NONMAIN_CAP = 80; // 非 main zone 阈值

// ====== 逃逸豁免：路径检查 ======
// 检查 agent → task1 → task2 的 Bresenham 直线路径是否：
//   ✅ 不穿墙
//   ✅ 不经过 main zone 热度 ≥ 35
//   ✅ 不经过非 main zone 热度 ≥ 80
// 返回 true=路径可用，false=不可用
inline bool check_connection_path(
    int agent_loc,
    const std::vector<int>& task_locs,
    const std::vector<int>& zone_density,
    const std::vector<int>& zone_is_main,
    const std::vector<int>& grid_map,
    int cols,
    int nzc)
{
    int max_z = (int)zone_density.size();
    int ax = agent_loc % cols, ay = agent_loc / cols;
    int az = (ay / 24) * nzc + (ax / 24);
    if (az < 0 || az >= max_z) return false;

    // 前置条件：agent 当前 main zone < 35
    if (zone_density[az] >= EX_MAIN_CAP) return false;

    for (int loc : task_locs) {
        int tx = loc % cols, ty = loc / cols;

        // ---- Bresenham 画线逐格检查 ----
        int cx = ax, cy = ay;
        int dx = std::abs(tx - ax), sx = ax < tx ? 1 : -1;
        int dy = -std::abs(ty - ay), sy = ay < ty ? 1 : -1;
        int err = dx + dy;

        while (true) {
            // ① 穿墙？
            if (grid_map[cy * cols + cx] != 0) return false;

            // ② 当前格子所在 zone 的检查
            int zc = (cy / 24) * nzc + (cx / 24);
            if (zc != az && zc >= 0 && zc < max_z) {
                int d = zone_density[zc];
                if (zone_is_main[zc]) {
                    if (d >= EX_MAIN_CAP) return false;
                } else {
                    if (d >= EX_NONMAIN_CAP) return false;
                }
            }

            if (cx == tx && cy == ty) break;

            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }

        ax = tx; ay = ty;
    }

    return true;
}

// ====== 热区豁免函数 ======
// agent所在zone密度>=ZONE_HARD_CAP → 是热区，应豁免
inline bool is_hot_zone_exempt(int loc, SharedEnvironment* env) {
    if (loc < 0) return false;
    int n24c = (env->cols + 23) / 24;
    int zi = (loc / env->cols / 24) * n24c + (loc % env->cols / 24);
    return (zi >= 0 && zi < (int)Centerline::g_zone24_density.size()
            && Centerline::g_zone24_density[zi] >= ZONE_HARD_CAP);
}

#endif // CONNECTION_EXEMPT_H
