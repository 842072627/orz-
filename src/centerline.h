// centerline.h - V3: BFS path + zoneâ†’main range + 2D heatmap
// All density data comes from 2D zone heatmap (g_zone24_density).
// 1D main_dist is used only for direction and zone-range overlap.
#pragma once
#include "SharedEnv.h"
#include "config.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <climits>
#include <cstdio>
#include <map>
#include "pibt_lock.h"
#include "bfs_table.h"

namespace Centerline {

// ====== State ======
static bool g_ready = false;
static int g_rows = 0, g_cols = 0;
static int g_total_len = 0;
static std::vector<int> g_main_dist;        // [loc] = main distance
static std::vector<int> g_branch_dist;      // [loc] = branch distance
static std::vector<int> g_centerline_cells;  // centerline cell locations
static std::vector<int> g_zone24_density;   // 2D heatmap: [zid] = live agent count
static int g_n24c = 0, g_n24r = 0;
static std::vector<int> g_zone_is_main;     // [zid] = 1 if centerline passes through

// ====== Per-cell corridor width (16-direction normalized, uint16_t) ======
// g_cell_width[loc] = normalized minimum pair span, 0xFFFF = wall
static std::vector<uint16_t> g_cell_width;

// ====== Zone->main range (maps each zone to its centerline main_dist range) ======
struct ZoneMainRange {
    int zone_id;      // 24x24 zone id
    int min_m, max_m; // centerline cells' main_dist range in this zone
};
static std::vector<ZoneMainRange> g_zone_ranges;

// ====== 1D main point agent tracking (prev tick main_dist for flow) ======


// ====== 1D main point system (MP_STEP=16 sampling along centerline) ======
static constexpr int MP_STEP = 16;           // step between main points
static int g_mp_count = 0;                   // number of main points
static std::vector<int> g_mp_loc;            // [si] = physical location of main point
static std::vector<int> g_mp_density;        // [si] = agents in 24x24 square centered on main point
static std::vector<int> g_mp_main_dist;      // [si] = main_dist of this main point
static std::vector<int> g_mp_width_p10;        // [si] = P10 corridor width (cells) per MP

// ====== Traffic prediction segment ======
#if USE_TRAFFIC_PREDICTION
enum SegState {
    SEG_IDLE,
    SEG_RIGHT_FLOW,
    SEG_LEFT_FLOW,
    SEG_RIGHT_CLEANUP,
    SEG_LEFT_CLEANUP,
};

struct TrafficSegment {
    int si_lo, si_hi;         // MP range [lo, hi)
    int p10_width;             // worst (min) width in this segment
    bool is_narrow;            // p10_width <= NARROW_WIDTH_MAX
    bool is_very_wide;         // p10_width > WIDE_BREAK (acts as group separator)
    int capacity;
    
    // Queue zone entrance (main_dist)
    int entrance_left_m;       // left (small main_dist) entrance
    int entrance_right_m;      // right (large main_dist) entrance
    
    // Predicted traffic (from predict_trajectory)
    int right_count;           // right-going agents predicted
    int left_count;            // left-going agents predicted
    int right_early, right_late;  // arrival tick range
    int left_early, left_late;
    
    // Narrow corridor state machine
    int flow_dir;              // +1 = right, -1 = left, 0 = none
    SegState state;
    int cleanup_ticks;         // remaining cleanup ticks
    int entrance_queue_right;  // agents buffered at left entrance
    int entrance_queue_left;   // agents buffered at right entrance
    int inside_count;          // agents currently inside segment
    int inside_right;          // agents going right inside segment
    int inside_left;           // agents going left inside segment
    int consecutive_no_exit;   // ticks with no progress (blockage detection)
    bool is_blocked;
    int group_id;              // -1=unassigned, >=0=group id (same direction enforced)
};

static std::vector<TrafficSegment> g_segments;
static int g_current_tick = 0;
#endif

// ====== BFS distance-to-wall ======
static void compute_dtw(const SharedEnvironment* env, std::vector<int>& dtw) {
    int total = env->rows * env->cols;
    dtw.assign(total, INT_MAX/2);
    std::queue<int> q;
    for (int i = 0; i < total; i++)
        if (env->map[i] != 0) { dtw[i] = 0; q.push(i); }
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int nd = dtw[cur] + 1;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total && env->map[n] == 0 && nd < dtw[n])
                { dtw[n] = nd; q.push(n); }
    }
}

// ====== A* widest path (robust version using BFS from Python heatmap_gen) ======
static void widest_path(const SharedEnvironment* env, const std::vector<int>& dtw,
                         int sx, int sy, int gx, int gy, std::vector<int>& path) {
    int total = env->rows * env->cols;
    int sloc = sy * g_cols + sx, gloc = gy * g_cols + gx;
    
    // BFS to find shortest path first
    std::vector<int> came_from(total, -1);
    std::queue<int> q;
    q.push(sloc);
    came_from[sloc] = sloc;
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        if (cur == gloc) break;
        int r = cur / g_cols, c = cur % g_cols;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh) {
            if (n >= 0 && n < total && env->map[n] == 0 && came_from[n] == -1) {
                came_from[n] = cur;
                q.push(n);
            }
        }
    }
    
    // Reconstruct shortest path
    path.clear();
    if (came_from[gloc] == -1) { path.push_back(sloc); path.push_back(gloc); return; }
    int cur = gloc;
    while (cur != sloc) { path.push_back(cur); cur = came_from[cur]; }
    path.push_back(sloc);
    std::reverse(path.begin(), path.end());
    
    // Widen: for each cell on shortest path with dtw <= 2, try to find a better (wider) cell
    // nearby with same shortest-path distance. This is a simple local optimization.
    int w = g_cols;
    for (size_t i = 1; i + 1 < path.size(); i++) {
        int loc = path[i];
        if (dtw[loc] > 2) continue;
        int best_loc = loc, best_clr = dtw[loc];
        int r = loc / w, c = loc % w;
        // Search within radius 3 for a cell with higher DTW
        for (int dr = -3; dr <= 3; dr++) {
            for (int dc = -3; dc <= 3; dc++) {
                if (std::abs(dr) + std::abs(dc) < 1 || std::abs(dr) + std::abs(dc) > 3) continue;
                int nr = r + dr, nc = c + dc;
                int nloc = nr * w + nc;
                if (nr >= 0 && nr < env->rows && nc >= 0 && nc < w && env->map[nloc] == 0) {
                    if (dtw[nloc] > best_clr) { best_clr = dtw[nloc]; best_loc = nloc; }
                }
            }
        }
        path[i] = best_loc;
    }
    
    // Dedup consecutive duplicates
    auto end = std::unique(path.begin(), path.end());
    path.erase(end, path.end());
}

// ====== Smooth path (fallback) ======
static void smooth_path(const SharedEnvironment* env, const std::vector<int>& raw,
                         std::vector<int>& out) {
    int window = 1;
    out.clear();
    for (size_t i = 0; i < raw.size(); i++) {
        int s = std::max(0, (int)i - window);
        int e = std::min((int)raw.size() - 1, (int)i + window);
        int sx = 0, sy = 0, cnt = 0;
        for (int j = s; j <= e; j++) {
            int r = raw[j] / g_cols, c = raw[j] % g_cols;
            sx += c; sy += r; cnt++;
        }
        int ax = std::max(0, std::min(g_cols-1, sx/cnt));
        int ay = std::max(0, std::min(g_rows-1, sy/cnt));
        int best_d = 999, best_loc = ay * g_cols + ax;
        for (int dy = -5; dy <= 5; dy++)
            for (int dx = -5; dx <= 5; dx++) {
                int nx = ax+dx, ny = ay+dy;
                if (nx>=0 && nx<g_cols && ny>=0 && ny<g_rows && env->map[ny*g_cols+nx]==0) {
                    int d = std::abs(dx)+std::abs(dy);
                    if (d < best_d) { best_d = d; best_loc = ny*g_cols+nx; }
                }
            }
        if (out.empty() || out.back() != best_loc) out.push_back(best_loc);
    }
}

// ====== Project loc -> (main, branch) ======
inline void project(int loc, int& main_dist, int& branch_dist) {
    if (!g_ready || loc < 0 || loc >= (int)g_branch_dist.size()) {
        main_dist = -1; branch_dist = -1; return;
    }
    branch_dist = g_branch_dist[loc];
    main_dist = (branch_dist >= 0) ? g_main_dist[loc] : -1;
}

// ====== Build zoneâ†’main range map ======
inline void build_zone_ranges() {
    std::map<int, std::pair<int,int>> zm_range;
    for (int loc : g_centerline_cells) {
        if (loc < 0) continue;
        int m = g_main_dist[loc];
        int r = loc / g_cols, c = loc % g_cols;
        int zid = (r / 24) * g_n24c + (c / 24);
        auto& p = zm_range[zid];
        if (p.first == 0 || m < p.first) p.first = m;
        if (m > p.second) p.second = m;
    }
    g_zone_ranges.clear();
    for (auto& kv : zm_range) {
        int zid = kv.first;
        g_zone_ranges.push_back({zid, kv.second.first, kv.second.second});
    }
    std::sort(g_zone_ranges.begin(), g_zone_ranges.end(),
        [](const ZoneMainRange& a, const ZoneMainRange& b) { return a.min_m < b.min_m; });
}

// Forward declaration for per-cell width computation
inline void compute_cell_width(const SharedEnvironment* env);

// ====== Forward declarations for traffic prediction ======
#if USE_TRAFFIC_PREDICTION
inline void init_segments();
inline void debug_traffic_csv();
#endif

// ====== Init: BFS path + wall correction + build zone ranges ======
inline void init(const SharedEnvironment* env) {
    g_rows = env->rows; g_cols = env->cols;
    g_n24c = env->cols / 24 + 1; g_n24r = env->rows / 24 + 1;
    int total = g_rows * g_cols;
    
    // Distance to wall
    std::vector<int> dtw;
    compute_dtw(env, dtw);
    
    // Find leftmost / rightmost walkable cells
    int start_loc = -1, goal_loc = -1, best_sc = -1, best_gc = -1;
    int min_x = g_cols, max_x = 0;
    for (int i = 0; i < total; i++) {
        if (env->map[i] != 0) continue;
        int c = i % g_cols;
        if (c < min_x || (c == min_x && dtw[i] > best_sc))
            { min_x = c; start_loc = i; best_sc = dtw[i]; }
        if (c > max_x || (c == max_x && dtw[i] > best_gc))
            { max_x = c; goal_loc = i; best_gc = dtw[i]; }
    }
    if (start_loc < 0 || goal_loc < 0) { return; }
    
    std::vector<int> base_path;
#if USE_MP_CENTERLINE
    // Phase 1: A* widest path (matches heatmap analysis compute_centerline)
    widest_path(env, dtw, start_loc % g_cols, start_loc / g_cols,
                goal_loc % g_cols, goal_loc / g_cols, base_path);
    { std::vector<int> tmp; smooth_path(env, base_path, tmp); base_path.swap(tmp); }
#else
    // Original: BFS shortest path (preserved, can revert by setting USE_MP_CENTERLINE=0)
    std::vector<int> came_from(total, -1);
    std::queue<int> bfs_q;
    bfs_q.push(start_loc);
    came_from[start_loc] = start_loc;
    int found_goal = -1;
    while (!bfs_q.empty()) {
        int cur = bfs_q.front(); bfs_q.pop();
        if (cur == goal_loc) { found_goal = cur; break; }
        int r = cur / g_cols, c = cur % g_cols;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total && env->map[n] == 0 && came_from[n] == -1)
                { came_from[n] = cur; bfs_q.push(n); }
    }
    if (found_goal < 0) {
        widest_path(env, dtw, start_loc % g_cols, start_loc / g_cols,
                    goal_loc % g_cols, goal_loc / g_cols, base_path);
        { std::vector<int> tmp; smooth_path(env, base_path, tmp); base_path.swap(tmp); }
    } else {
        int cur = goal_loc;
        while (cur != start_loc) { base_path.push_back(cur); cur = came_from[cur]; }
        base_path.push_back(start_loc);
        std::reverse(base_path.begin(), base_path.end());
    }
#endif
    
    // Wall correction: for DTW <= 2 cells, scan radius 5, snap to best zone
    std::unordered_map<int, int> corrected_map; // path_idx -> corrected loc
    for (size_t i = 0; i < base_path.size(); i++) {
        int loc = base_path[i];
        if (dtw[loc] > 2) continue;
        int pr = loc / g_cols, pc = loc % g_cols;
        std::unordered_map<int, int> zone_hits;
        for (int dr = -5; dr <= 5; dr++) {
            for (int dc = -5; dc <= 5; dc++) {
                if (std::abs(dr) + std::abs(dc) < 1 || std::abs(dr) + std::abs(dc) > 5) continue;
                int nr = pr + dr, nc = pc + dc;
                if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols && env->map[nr*g_cols+nc] == 0) {
                    int zid = (nr / 24) * g_n24c + (nc / 24);
                    zone_hits[zid]++;
                }
            }
        }
        int best_zone = -1, best_cnt = -1;
        for (auto& kv : zone_hits)
            if (kv.second > best_cnt) { best_cnt = kv.second; best_zone = kv.first; }
        if (best_zone >= 0) {
            int replacement = loc, best_d = 999999;
            for (size_t j = 0; j < base_path.size(); j++) {
                int pz = (base_path[j] / g_cols / 24) * g_n24c + (base_path[j] % g_cols / 24);
                if (pz == best_zone) {
                    int d = std::abs(base_path[j] / g_cols - pr) + std::abs(base_path[j] % g_cols - pc);
                    if (d < best_d) { best_d = d; replacement = base_path[j]; }
                }
            }
            corrected_map[(int)i] = replacement;
        }
    }
    
    // Build final centerline
    g_centerline_cells.clear();
    for (size_t i = 0; i < base_path.size(); i++) {
        int cell = base_path[i];
        if (dtw[cell] <= 2 && corrected_map.count((int)i))
            cell = corrected_map[(int)i];
        if (g_centerline_cells.empty() || g_centerline_cells.back() != cell)
            g_centerline_cells.push_back(cell);
    }
    
    // Main distances along centerline
    std::unordered_map<int, int> cl_main;
    int cumul = 0;
    cl_main[g_centerline_cells[0]] = cumul;
    for (size_t i = 1; i < g_centerline_cells.size(); i++) {
        int r1 = g_centerline_cells[i-1]/g_cols, c1 = g_centerline_cells[i-1]%g_cols;
        int r2 = g_centerline_cells[i]/g_cols, c2 = g_centerline_cells[i]%g_cols;
        cumul += (int)std::round(std::sqrt((double)((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1))));
        cl_main[g_centerline_cells[i]] = cumul;
    }
    g_total_len = cumul;
    
    // BFS from centerline -> branch_dist + main_dist
    g_branch_dist.assign(total, -1);
    g_main_dist.assign(total, -1);
    std::queue<int> seed_q;
    for (size_t i = 0; i < g_centerline_cells.size(); i++) {
        int loc = g_centerline_cells[i];
        g_branch_dist[loc] = 0;
        g_main_dist[loc] = cl_main[loc];
        seed_q.push(loc);
        if (i + 1 < g_centerline_cells.size()) {
            int nloc = g_centerline_cells[i+1];
            int r1 = loc / g_cols, c1 = loc % g_cols;
            int r2 = nloc / g_cols, c2 = nloc % g_cols;
            int m1 = cl_main[loc], m2 = cl_main[nloc];
            for (int k = 1; k < 20; k++) {
                float t = (float)k / 20.0f;
                int ir = (int)std::round(r1 + (r2 - r1) * t);
                int ic = (int)std::round(c1 + (c2 - c1) * t);
                int iloc = ir * g_cols + ic;
                if (iloc >= 0 && iloc < total && env->map[iloc] == 0 && g_branch_dist[iloc] == -1) {
                    g_branch_dist[iloc] = 0;
                    g_main_dist[iloc] = (int)std::round(m1 + (m2 - m1) * t);
                    seed_q.push(iloc);
                }
            }
        }
    }
    while (!seed_q.empty()) {
        int cur = seed_q.front(); seed_q.pop();
        int nd = g_branch_dist[cur] + 1;
        int nmain = g_main_dist[cur];
        int r = cur / g_cols, c = cur % g_cols;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total && env->map[n] == 0 && g_branch_dist[n] == -1)
                { g_branch_dist[n] = nd; g_main_dist[n] = nmain; seed_q.push(n); }
    }
    
    // Zone density init
    g_zone24_density.assign(g_n24c * g_n24r, 0);
    
    
    // Mark main-line zones
    g_zone_is_main.assign(g_n24c * g_n24r, 0);
    for (int clloc : g_centerline_cells) {
        int r = clloc / g_cols, c = clloc % g_cols;
        int zid = (r / 24) * g_n24c + (c / 24);
        if (zid >= 0 && zid < (int)g_zone_is_main.size())
            g_zone_is_main[zid] = 1;
    }
    
    // Initialize 1D main points: sample centerline at MP_STEP intervals
    g_mp_count = (int)g_centerline_cells.size() / MP_STEP + 1;
    g_mp_loc.assign(g_mp_count, -1);
    g_mp_density.assign(g_mp_count, 0);
    g_mp_main_dist.assign(g_mp_count, -1);
    g_mp_width_p10.assign(g_mp_count, 999);
    
    // Compute tangents for centerline cells
    int n_cl = (int)g_centerline_cells.size();
    std::vector<int> cl_tdx(n_cl, 0), cl_tdy(n_cl, 0);
    for (int i = 0; i < n_cl; i++) {
        int r1, c1, r2, c2;
        if (i == 0) {
            r1 = g_centerline_cells[0] / g_cols; c1 = g_centerline_cells[0] % g_cols;
            r2 = g_centerline_cells[1] / g_cols; c2 = g_centerline_cells[1] % g_cols;
        } else {
            r1 = g_centerline_cells[i-1] / g_cols; c1 = g_centerline_cells[i-1] % g_cols;
            r2 = g_centerline_cells[i] / g_cols; c2 = g_centerline_cells[i] % g_cols;
        }
        cl_tdx[i] = c2 - c1;  // x difference (col)
        cl_tdy[i] = r2 - r1;  // y difference (row)
    }

    // Pre-compute per-cell corridor width for p10 scanning
    compute_cell_width(env);

    for (int si = 0; si < g_mp_count; si++) {
        int idx = si * MP_STEP;
        if (idx >= n_cl) idx = n_cl - 1;
        g_mp_loc[si] = g_centerline_cells[idx];
        auto md_it = cl_main.find(g_mp_loc[si]);
        g_mp_main_dist[si] = (md_it != cl_main.end()) ? md_it->second : -1;
#if USE_MP_WIDTH
        if (g_mp_main_dist[si] >= 0) {
            int target_m = g_mp_main_dist[si];
            int half_range = MP_STEP / 2;
            std::vector<int> cw_vals;
            for (int cl_loc : g_centerline_cells) {
                auto cl_it = cl_main.find(cl_loc);
                if (cl_it == cl_main.end()) continue;
                if (std::abs(cl_it->second - target_m) <= half_range) {
                    int cw = g_cell_width[cl_loc];
                    if (cw < 500 && cw > 0) cw_vals.push_back(cw);
                }
            }
            if (!cw_vals.empty()) {
                std::sort(cw_vals.begin(), cw_vals.end());
                int p10 = cw_vals[cw_vals.size() / 10];
                g_mp_width_p10[si] = p10;
            }
        }
#endif
    }

    build_zone_ranges();

    // BFS distance table (off-main comps always; full matrix if USE_BFS_TABLE=1)
    BFSTable::init(env, g_zone_is_main, g_n24c);

#if USE_TRAFFIC_PREDICTION
    init_segments();
#if USE_DIAG_LOG
    debug_traffic_csv();
#endif
#endif

    g_ready = true;
}

// ====== Future flow removed, replaced by TrafficSegment prediction ======
// ====== 2D zone density (tick call) ======
inline void update_zone24_density(const SharedEnvironment* env) {
    int n24c = (env->cols + 23) / 24, n24r = (env->rows + 23) / 24;
    if ((int)g_zone24_density.size() != n24c * n24r)
        g_zone24_density.assign(n24c * n24r, 0);
    else
        std::fill(g_zone24_density.begin(), g_zone24_density.end(), 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        if (USE_DENSITY_EXCLUDE_IDLE) {
            if (env->curr_task_schedule[a] < 0) continue;
            if (a < PIBT_Lock::MAX_AGENTS && PIBT_Lock::g_agent_blocked[a]) continue;
        }
        int r = loc / env->cols, c = loc % env->cols;
        int zi = (r / 24) * n24c + (c / 24);
        if (zi >= 0 && zi < (int)g_zone24_density.size())
            g_zone24_density[zi]++;
    }
}

// ====== 1D main point density (tick call) ======
inline void update_mp_density(const SharedEnvironment* env) {
    const int HW = 12;  // half-width of 24x24 square
    std::fill(g_mp_density.begin(), g_mp_density.end(), 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        if (USE_DENSITY_EXCLUDE_IDLE) {
            if (env->curr_task_schedule[a] < 0) continue;
            if (a < PIBT_Lock::MAX_AGENTS && PIBT_Lock::g_agent_blocked[a]) continue;
        }
        int ar = loc / env->cols, ac = loc % env->cols;
        for (int si = 0; si < g_mp_count; si++) {
            int mloc = g_mp_loc[si];
            if (mloc < 0) continue;
            int mr = mloc / env->cols, mc = mloc % env->cols;
            if (std::abs(ar - mr) < HW && std::abs(ac - mc) < HW)
                g_mp_density[si]++;
        }
    }
}

// update_mp_direction removed — replaced by TrafficSegment prediction

// ====== 16-direction per-cell corridor width (static, computed once at init) ======
// Ported from cell_width_map_v3.py: 16 pairs, step-length normalized, dead-end skip
inline void compute_cell_width(const SharedEnvironment* env) {
    int total = g_rows * g_cols;
    g_cell_width.assign(total, 0xFFFF);
    
    // 16 direction pairs (dx1,dy1,dx2,dy2) + precomputed step lengths
    static const int dp[16][4] = {
        {0,-1,0,1}, {-1,0,1,0},
        {1,-1,-1,1}, {1,1,-1,-1},
        {2,-1,-2,1}, {2,1,-2,-1},
        {1,-2,-1,2}, {1,2,-1,-2},
        {3,-1,-3,1}, {3,1,-3,-1},
        {1,-3,-1,3}, {1,3,-1,-3},
        {3,-2,-3,2}, {3,2,-3,-2},
        {2,-3,-2,3}, {2,3,-2,-3},
    };
    static const float ps[16] = {
        1.0f, 1.0f, 1.414f, 1.414f, 2.236f, 2.236f, 2.236f, 2.236f,
        3.162f, 3.162f, 3.162f, 3.162f, 3.606f, 3.606f, 3.606f, 3.606f
    };
    
    for (int y = 0; y < g_rows; y++) {
        for (int x = 0; x < g_cols; x++) {
            int loc = y * g_cols + x;
            if (env->map[loc] != 0) continue;
            float min_span = 1e9f;
            for (int pi = 0; pi < 16; pi++) {
                int dx1 = dp[pi][0], dy1 = dp[pi][1];
                int dx2 = dp[pi][2], dy2 = dp[pi][3];
                int c1 = 0;
                for (int k = 1; k < 100; k++) {
                    int nx = x + dx1 * k, ny = y + dy1 * k;
                    if (nx < 0 || nx >= g_cols || ny < 0 || ny >= g_rows) break;
                    if (env->map[ny * g_cols + nx] != 0) break;
                    c1++;
                }
                int c2 = 0;
                for (int k = 1; k < 100; k++) {
                    int nx = x + dx2 * k, ny = y + dy2 * k;
                    if (nx < 0 || nx >= g_cols || ny < 0 || ny >= g_rows) break;
                    if (env->map[ny * g_cols + nx] != 0) break;
                    c2++;
                }
                if (c1 == 0 && c2 == 0) continue;
                float span = c1 * ps[pi] + c2 * ps[pi] + 1.0f;
                if (span < min_span) min_span = span;
            }
            g_cell_width[loc] = (min_span < 1e9f)
                ? (uint16_t)(int)::round(min_span)
                : 1;
        }
    }
#if USE_DIAG_LOG
    printf("CELL_WIDTH: computed for %d walkable cells\n", (int)g_cell_width.size());
#endif
}

// ====== Traffic prediction: segment initialization ======
#if USE_TRAFFIC_PREDICTION
inline void init_segments() {
    g_segments.clear();
    if (g_mp_count <= 0) return;
    
    int seg_start = 0;
    for (int si = 1; si <= g_mp_count; si++) {
        bool split = false;
        if (si == g_mp_count) split = true;
        else if (si > 0 && std::abs(g_mp_width_p10[si] - g_mp_width_p10[seg_start]) >= 3)
            split = true;
        else if (si - seg_start >= 4)
            split = true;
        
        if (split) {
            int seg_len = si - seg_start;
            if (seg_len <= 0) { seg_start = si; continue; }
            
            int min_w = 9999;
            for (int s = seg_start; s < si; s++)
                if (g_mp_width_p10[s] < min_w) min_w = g_mp_width_p10[s];
            
            TrafficSegment seg;
            seg.si_lo = seg_start;
            seg.si_hi = si;
            seg.p10_width = min_w;
            seg.is_narrow = (min_w <= NARROW_WIDTH_MAX);
            seg.is_very_wide = (min_w > 15);
            seg.group_id = -1;
            seg.capacity = (seg.is_narrow) ? 1 : (min_w / 2);
            
            int prev_si = std::max(0, seg_start - 1);
            seg.entrance_left_m = g_mp_main_dist[prev_si];
            seg.entrance_right_m = g_mp_main_dist[std::min(si, g_mp_count - 1)];
            
            seg.right_count = seg.left_count = 0;
            seg.right_early = seg.left_early = 999999;
            seg.right_late = seg.left_late = 0;
            seg.flow_dir = 0;
            seg.state = SEG_IDLE;
            seg.cleanup_ticks = 0;
            seg.entrance_queue_right = seg.entrance_queue_left = 0;
            seg.inside_count = 0;
            seg.consecutive_no_exit = 0;
            seg.is_blocked = false;
            
            g_segments.push_back(seg);
            seg_start = si;
        }
    }
    
    // Compute groups: merge narrow segs within 200 main_dist, unless very-wide(width>15) in between
    // All segs in the same group must have same flow_dir
    static constexpr int GROUP_THRESHOLD = 200;
    static constexpr int WIDE_BREAK = 15;
    int cur_group = 0;
    for (int i = 0; i < (int)g_segments.size(); i++) {
        if (g_segments[i].group_id >= 0) continue;
        if (!g_segments[i].is_narrow) { g_segments[i].group_id = -1; continue; }
        
        // Scan forward to build group
        for (int j = i; j < (int)g_segments.size(); j++) {
            if (!g_segments[j].is_narrow) continue;
            if (g_segments[j].group_id >= 0) continue;
            
            // Check if there's a very-wide segment between the group and this candidate
            bool has_break = false;
            int last_m = g_segments[i].si_lo;
            int curr_m = g_segments[j].si_lo;
            if (std::abs(last_m - curr_m) * 16 > GROUP_THRESHOLD) continue;
            
            for (int k = i; k < j; k++) {
                if (g_segments[k].is_very_wide) {
                    has_break = true; break;
                }
            }
            if (has_break) break;
            
            g_segments[j].group_id = cur_group;
        }
        cur_group++;
    }
    
    // Mark non-narrow segments as ungrouped
    for (auto& seg : g_segments) {
        if (!seg.is_narrow && seg.group_id == -1)
            seg.group_id = -1;
    }
}

// Predict trajectory for one agent along its task path
inline void predict_trajectory(int /*aid*/, int curr_loc,
                                const std::vector<int>& task_locs,
                                SharedEnvironment* env) {
    int curr_m, curr_b;
    project(curr_loc, curr_m, curr_b);
    if (curr_m < 0) return;
    
    int prev_m = curr_m;
    int cumul_dist = 0;
    
    for (int loc : task_locs) {
        int tm, tb;
        project(loc, tm, tb);
        if (tm < 0) continue;
        
        int seg_lo = std::min(prev_m, tm);
        int seg_hi = std::max(prev_m, tm);
        int dir = (tm > prev_m) ? +1 : -1;
        int seg_len = std::abs(tm - prev_m);
        
        for (auto& seg : g_segments) {
            int sm_lo = g_mp_main_dist[seg.si_lo];
            int sm_hi = g_mp_main_dist[std::min(seg.si_hi, g_mp_count - 1)];
            if (seg_hi <= sm_lo || seg_lo >= sm_hi) continue;
            
            int overlap_lo = std::max(seg_lo, sm_lo);
            int dist_to_seg = cumul_dist + std::abs(overlap_lo - prev_m);
            int arrival = (int)((float)dist_to_seg / PREDICT_SPEED);
            int exit = (int)((float)(dist_to_seg + seg_len) / PREDICT_SPEED);
            
            if (dir > 0) {
                seg.right_count++;
                if (arrival < seg.right_early) seg.right_early = arrival;
                if (exit > seg.right_late) seg.right_late = exit;
            } else {
                seg.left_count++;
                if (arrival < seg.left_early) seg.left_early = arrival;
                if (exit > seg.left_late) seg.left_late = exit;
            }
        }
        
        cumul_dist += seg_len;
        prev_m = tm;
    }
}

// Debug CSV
inline void debug_traffic_csv() {
    static FILE* f = nullptr;
    if (!f) {
        f = fopen("C:/Users/Administrator/.openclaw/workspace/traffic_prediction.csv", "w");
        fprintf(f, "tick,seg_id,si_lo,si_hi,width,is_narrow,"
                   "state,flow_dir,right_c,left_c,"
                   "right_early,right_late,left_early,left_late,"
                   "queue_r,queue_l,inside,in_r,in_l,blocked,cleanup\n");
    }
    for (int i = 0; i < (int)g_segments.size(); i++) {
        auto& s = g_segments[i];
        const char* st = "IDLE";
        switch (s.state) {
            case SEG_RIGHT_FLOW: st = "R_FLOW"; break;
            case SEG_LEFT_FLOW:  st = "L_FLOW"; break;
            case SEG_RIGHT_CLEANUP: st = "R_CLEAN"; break;
            case SEG_LEFT_CLEANUP:  st = "L_CLEAN"; break;
            default: break;
        }
        fprintf(f, "%d,%d,%d,%d,%d,%d,"
                   "%s,%d,%d,%d,"
                   "%d,%d,%d,%d,"
                   "%d,%d,%d,%d,%d,%d,%d\n",
                g_current_tick, i, s.si_lo, s.si_hi, s.p10_width, (int)s.is_narrow,
                st, s.flow_dir, s.right_count, s.left_count,
                s.right_early, s.right_late, s.left_early, s.left_late,
                s.entrance_queue_right, s.entrance_queue_left,
                s.inside_count, s.inside_right, s.inside_left, (int)s.is_blocked, s.cleanup_ticks);
    }
}
#endif // USE_TRAFFIC_PREDICTION

} // namespace Centerline


