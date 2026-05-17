// narrow_passage.h — 基于中心线切线的走廊宽度识别
// 精确映射 narrow_map_gen.py 的算法
#pragma once
#include "SharedEnv.h"
#include "centerline.h"
#include "config.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace NarrowPassage {

// ====== 走廊宽度分类（与 narrow_map_gen.py 一致） ======
enum CorridorClass {
    SUPER_NARROW = 5,   // ≤3 cells
    NARROW       = 4,   // 4-6 cells
    MODERATE     = 3,   // 7-12 cells
    WIDE         = 2,   // 13-20 cells
    VERY_WIDE    = 1    // >20 cells
};

// ====== 预计算数据 ======
static bool g_ready = false;
static int g_rows = 0, g_cols = 0;

// 每个中心线cell的切线方向（归一化）
static std::vector<int> g_tangent_dx;   // [cl_idx]
static std::vector<int> g_tangent_dy;   // [cl_idx]

// 每个中心线cell的垂直方向宽度（切线垂直方向量到墙）
static std::vector<int> g_perp_width;   // [cl_idx] = 走廊垂直宽度(cells)

// 每个中心线cell的走廊分类
static std::vector<CorridorClass> g_cls; // [cl_idx]

// 每个zone的宽度数据
static std::vector<CorridorClass> g_zone_cls;  // [zid]
static std::vector<int> g_zone_min_width;       // [zid] zone内最窄宽度
static std::vector<int> g_zone_avg_width;       // [zid] zone内平均宽度
static std::vector<int> g_zone_centerline_cells; // [zid] zone内中心线cell数

// Dead-end zone detection moved to centerline.h (is_dead_end_zone)

// ====== 边缘cell（edge lane slot）预计算 ======
// 每个被管控的zone按等级提供边缘slot
struct EdgeSlot {
    int loc;           // cell location
    int zx, zy;        // zone coords
    bool is_right;     // true=右侧（相对lock_dir方向）, false=左侧
    int layer;         // 第几层（1=最外层，2=次外层）
};
static std::vector<EdgeSlot> g_edge_slots;        // 所有edge slot
static std::vector<int> g_slot_assign;             // [slot_id] → agent_id, -1=空

// loc→slot索引映射
static std::vector<int> g_loc_to_slot;             // [loc] → slot_id, -1=不是slot

// ====== 切线方向计算（与 narrow_map_gen.py 完全一致） ======
inline int normalize_dir(int d) {
    return (d > 0) ? 1 : (d < 0) ? -1 : 0;
}

// ====== 垂直宽度测量（核心算法） ======
// 从(cx,cy)沿垂直方向(tdx,tdy的垂直)量到墙的距离
inline int measure_perp_width(const SharedEnvironment* env, int cx, int cy, int tdx, int tdy) {
    int h = env->rows, w = env->cols;
    int px = -tdy, py = tdx;  // 垂直方向（左旋90度）
    
    int d1 = 0, d2 = 0;
    // 正方向走
    for (int k = 1; k < 100; k++) {
        int nx = cx + px * k, ny = cy + py * k;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) break;
        if (env->map[ny * w + nx] != 0) break;
        d1++;
    }
    // 负方向走
    for (int k = 1; k < 100; k++) {
        int nx = cx - px * k, ny = cy - py * k;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) break;
        if (env->map[ny * w + nx] != 0) break;
        d2++;
    }
    return d1 + d2 + 1;  // 两个方向+自身
}

// ====== 走廊分类 ======
inline CorridorClass classify_width(int w) {
    if (w <= 3)  return SUPER_NARROW;
    if (w <= 6)  return NARROW;
    if (w <= 12) return MODERATE;
    if (w <= 20) return WIDE;
    return VERY_WIDE;
}

// 死角检测已移至 centerline.h (compute_dead_end_zones)
// g_zone_is_dead_end 由 Centerline::is_dead_end_zone() 替代

// ====== 边缘slot生成（按zone，密集生成） ======
// M: 1层边道, W: 2层边道
// 对每个在管控zone内的中心线cell，沿垂直方向走到距离墙layer步处作为edge cell
inline void generate_edge_slots(const SharedEnvironment* env) {
    g_edge_slots.clear();
    int total = env->rows * env->cols;
    g_loc_to_slot.assign(total, -1);
    
    for (size_t ci = 0; ci < Centerline::g_centerline_cells.size(); ci++) {
        int loc = Centerline::g_centerline_cells[ci];
        int r = loc / env->cols, c = loc % env->cols;
        
        CorridorClass cls = g_cls[ci];
        int width = g_perp_width[ci];
        
        int layers = 0;
        if (cls == MODERATE) layers = 1;       // M: 1层
        else if (cls == WIDE) layers = 2;       // W: 2层
        else continue;  // SN/N/VW不生成
        
        int tdx = g_tangent_dx[ci], tdy = g_tangent_dy[ci];
        int px = -tdy, py = tdx;  // 垂直方向
        int zx = c / 24, zy = r / 24;
        
        // 左右两侧各生成layer层
        for (int layer = 1; layer <= layers; layer++) {
            int dist_from_wall = (width / 2) - layer;
            if (dist_from_wall < 1) continue;
            
            // 右侧
            int er = r + py * dist_from_wall;
            int ec = c + px * dist_from_wall;
            if (er >= 0 && er < env->rows && ec >= 0 && ec < env->cols && env->map[er * env->cols + ec] == 0) {
                int slot_loc = er * env->cols + ec;
                if (g_loc_to_slot[slot_loc] < 0) {
                    EdgeSlot es;
                    es.loc = slot_loc;
                    es.zx = zx; es.zy = zy;
                    es.is_right = true;
                    es.layer = layer;
                    g_edge_slots.push_back(es);
                    g_loc_to_slot[slot_loc] = (int)g_edge_slots.size() - 1;
                }
            }
            
            // 左侧
            int el = r - py * dist_from_wall;
            int ecol = c - px * dist_from_wall;
            if (el >= 0 && el < env->rows && ecol >= 0 && ecol < env->cols && env->map[el * env->cols + ecol] == 0) {
                int slot_loc = el * env->cols + ecol;
                if (g_loc_to_slot[slot_loc] < 0) {
                    EdgeSlot es;
                    es.loc = slot_loc;
                    es.zx = zx; es.zy = zy;
                    es.is_right = false;
                    es.layer = layer;
                    g_edge_slots.push_back(es);
                    g_loc_to_slot[slot_loc] = (int)g_edge_slots.size() - 1;
                }
            }
        }
    }
    
    g_slot_assign.assign(g_edge_slots.size(), -1);
    printf("[NARROW] Edge slots generated: %zu\n", g_edge_slots.size());
}

// ====== 初始化（预处理一次性） ======
inline void init(const SharedEnvironment* env) {
    g_rows = env->rows;
    g_cols = env->cols;
    
    if (!Centerline::g_ready) {
        printf("[NARROW] ERROR: centerline not ready\n");
        return;
    }
    
    const auto& cl = Centerline::g_centerline_cells;
    size_t ncl = cl.size();
    
    g_tangent_dx.assign(ncl, 0);
    g_tangent_dy.assign(ncl, 0);
    g_perp_width.assign(ncl, 0);
    g_cls.assign(ncl, VERY_WIDE);
    
    // Step 1: 计算每个中心线cell的切线方向
    for (size_t i = 0; i < ncl; i++) {
        int dx, dy;
        if (i == 0) {
            dx = cl[1] % g_cols - cl[0] % g_cols;
            dy = cl[1] / g_cols - cl[0] / g_cols;
        } else if (i == ncl - 1) {
            dx = cl[i] % g_cols - cl[i-1] % g_cols;
            dy = cl[i] / g_cols - cl[i-1] / g_cols;
        } else {
            dx = cl[i+1] % g_cols - cl[i-1] % g_cols;
            dy = cl[i+1] / g_cols - cl[i-1] / g_cols;
        }
        int nd = std::max(1, std::max(std::abs(dx), std::abs(dy)));
        g_tangent_dx[i] = dx / nd;
        g_tangent_dy[i] = dy / nd;
    }
    
    // Step 2: 计算每个中心线cell的垂直宽度
    for (size_t i = 0; i < ncl; i++) {
        int loc = cl[i];
        int r = loc / g_cols, c = loc % g_cols;
        int w = measure_perp_width(env, c, r, g_tangent_dx[i], g_tangent_dy[i]);
        g_perp_width[i] = w;
        g_cls[i] = classify_width(w);
    }
    
    // Step 3: 计算zone级别的宽度数据
    int n24c = Centerline::g_n24c;
    int n24r = Centerline::g_n24r;
    int nz = n24c * n24r;
    
    g_zone_cls.assign(nz, VERY_WIDE);
    g_zone_min_width.assign(nz, 999);
    g_zone_avg_width.assign(nz, 0);
    g_zone_centerline_cells.assign(nz, 0);
    
    std::vector<int> width_sum(nz, 0), cell_count(nz, 0);
    for (size_t i = 0; i < ncl; i++) {
        int loc = cl[i];
        int zid = (loc / g_cols / 24) * n24c + ((loc % g_cols) / 24);
        if (zid >= 0 && zid < nz) {
            int w = g_perp_width[i];
            if (w < g_zone_min_width[zid]) g_zone_min_width[zid] = w;
            width_sum[zid] += w;
            cell_count[zid]++;
        }
    }
    for (int z = 0; z < nz; z++) {
        g_zone_centerline_cells[z] = cell_count[z];
        if (cell_count[z] > 0) {
            g_zone_avg_width[z] = width_sum[z] / cell_count[z];
            g_zone_cls[z] = classify_width(g_zone_min_width[z] <= 0 ? 999 : g_zone_min_width[z]);
        }
    }
    
    // Step 4: 生成edge slot（死角检测在centerline.h中）
    generate_edge_slots(env);
    
    // 统计
    int counts[6] = {0};
    for (size_t i = 0; i < ncl; i++) {
        int c = (int)g_cls[i];
        if (c >= 1 && c <= 5) counts[c]++;
    }
    printf("[NARROW] Centerline cells: %zu | SN=%d N=%d M=%d W=%d VW=%d\n",
           ncl, counts[5], counts[4], counts[3], counts[2], counts[1]);
    printf("[NARROW] Zone min width: SN=%d N=%d M=%d W=%d VW=%d\n",
           (int)std::count(g_zone_cls.begin(), g_zone_cls.end(), SUPER_NARROW),
           (int)std::count(g_zone_cls.begin(), g_zone_cls.end(), NARROW),
           (int)std::count(g_zone_cls.begin(), g_zone_cls.end(), MODERATE),
           (int)std::count(g_zone_cls.begin(), g_zone_cls.end(), WIDE),
           (int)std::count(g_zone_cls.begin(), g_zone_cls.end(), VERY_WIDE));
    
    g_ready = true;
}

// ====== 获取zone的管控热度阈值 ======
inline int get_control_threshold(int zid) {
    if (zid < 0 || zid >= (int)g_zone_cls.size()) return 999;
    CorridorClass cls = g_zone_cls[zid];
    // N/SN: 只要有方向锁就管控（阈值=0）
    if (cls == SUPER_NARROW || cls == NARROW) return 0;
    // M: heat≥30 管控
    if (cls == MODERATE) return 30;
    // W: heat≥40 管控
    if (cls == WIDE) return 40;
    // VW: 不管控
    return 999;
}

} // namespace NarrowPassage
