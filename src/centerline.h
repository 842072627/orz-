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

namespace Centerline {

// Direction lock uses ZONE_HARD_CAP (from config.h) as activation threshold

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

// ====== Zone->main range (maps each zone to its centerline main_dist range) ======
struct ZoneMainRange {
    int zone_id;      // 24x24 zone id
    int min_m, max_m; // centerline cells' main_dist range in this zone
};
static std::vector<ZoneMainRange> g_zone_ranges;

// ====== 1D main point agent tracking (prev tick main_dist for flow) ======
static std::vector<int> g_mp_agent_last_main;   // [aid] = last tick main distance

// ====== 1D main point system (MP_STEP=16 sampling along centerline) ======
#if USE_MP_DENSITY
static constexpr int MP_STEP = 16;           // step between main points
static int g_mp_count = 0;                   // number of main points
static std::vector<int> g_mp_loc;            // [si] = physical location of main point
static std::vector<int> g_mp_density;        // [si] = agents in 24x24 square centered on main point
static std::vector<int> g_mp_flow_right;     // [si] = flow towards larger main_dist
static std::vector<int> g_mp_flow_left;      // [si] = flow towards smaller main_dist
static std::vector<int> g_mp_dir;            // [si] = majority flow direction (+1/-1/0)
static std::vector<int> g_mp_lock_dir;       // [si] = direction lock (+1/-1/0)
static std::vector<int> g_mp_main_dist;      // [si] = main_dist of this main point
static std::vector<int> g_mp_threshold;      // [si] = density threshold for direction lock (15/20/30)
static std::vector<float> g_mp_future_right;   // [si] = distance-weighted future flow right
static std::vector<float> g_mp_future_left;    // [si] = distance-weighted future flow left
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

// ====== Perpendicular corridor width at a cell ======
// Scans perpendicular to tangent direction, counts steps to wall on each side.
// Skips "hole" cells (both tangent sides are walls within 1 step).
static int perp_width_at(const SharedEnvironment* env, int loc, int tdx, int tdy) {
    int px = -tdy, py = tdx;  // perpendicular direction
    int cols = env->cols, rows = env->rows;
    int gx = loc % cols, gy = loc / cols;
    
    // is_hole: if both tangent sides are walls within 1 step from (x,y) → hole
    auto is_hole = [&](int x, int y) -> bool {
        for (int s : {-1, 1}) {
            int nx = x + tdx * s, ny = y + tdy * s;
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows && env->map[ny * cols + nx] == 0)
                return false;  // open on at least one side
        }
        return true;
    };
    
    int d1 = 0, d2 = 0;
    for (int side : {-1, 1}) {
        int d = 0;
        for (int k = 1; k < 100; k++) {
            int nx = gx + px * side * k, ny = gy + py * side * k;
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || env->map[ny * cols + nx] != 0)
                break;
            if (is_hole(nx, ny)) continue;
            d++;
        }
        if (side == -1) d1 = d; else d2 = d;
    }
    return d1 + d2 + 1;
}

// Classify corridor width into threshold
static int classify_threshold(int width) {
    if (width <= 8)  return 15;  // SUPER_NARROW + NARROW
    if (width <= 12) return 20;  // MODERATE
    return 30;  // WIDE + VERY_WIDE
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

// ====== Init: BFS path + wall correction + build zone ranges ======
void init(const SharedEnvironment* env) {
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
    
#if USE_MP_DENSITY
    // Initialize 1D main points: sample centerline at MP_STEP intervals
    g_mp_count = (int)g_centerline_cells.size() / MP_STEP + 1;
    g_mp_loc.assign(g_mp_count, -1);
    g_mp_density.assign(g_mp_count, 0);
    g_mp_flow_right.assign(g_mp_count, 0);
    g_mp_flow_left.assign(g_mp_count, 0);
    g_mp_dir.assign(g_mp_count, 0);
    g_mp_lock_dir.assign(g_mp_count, 0);
    g_mp_main_dist.assign(g_mp_count, -1);
    g_mp_threshold.assign(g_mp_count, ZONE_HARD_CAP);
    g_mp_future_right.assign(g_mp_count, 0.0f);
    g_mp_future_left.assign(g_mp_count, 0.0f);
    
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
    
    for (int si = 0; si < g_mp_count; si++) {
        int idx = si * MP_STEP;
        if (idx >= n_cl) idx = n_cl - 1;
        g_mp_loc[si] = g_centerline_cells[idx];
        auto md_it = cl_main.find(g_mp_loc[si]);
        g_mp_main_dist[si] = (md_it != cl_main.end()) ? md_it->second : -1;
        // Threshold stays at ZONE_HARD_CAP (15) for all main points
        // Phase 4 (USE_MP_WIDTH) would enable width-adaptive thresholds here
    }
#endif

    build_zone_ranges();
    g_ready = true;
}

// ====== Future flow stub (data filled by strategy_centerline) ======
inline void compute_future_direction() {}

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
        int r = loc / env->cols, c = loc % env->cols;
        int zi = (r / 24) * n24c + (c / 24);
        if (zi >= 0 && zi < (int)g_zone24_density.size())
            g_zone24_density[zi]++;
    }
}

// Zone direction: now handled by 1D MP system (update_mp_direction)
// Old 2D zone direction system removed in cleanup.

// ====== 1D main point density (tick call) ======
#if USE_MP_DENSITY
inline void update_mp_density(const SharedEnvironment* env) {
    const int HW = 12;  // half-width of 24x24 square
    std::fill(g_mp_density.begin(), g_mp_density.end(), 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
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
#endif

// ====== 1D main point direction (tick call) ======
#if USE_MP_DENSITY
inline void update_mp_direction(const SharedEnvironment* env) {
    // 1. Flow tracking per main point
    std::fill(g_mp_flow_right.begin(), g_mp_flow_right.end(), 0);
    std::fill(g_mp_flow_left.begin(), g_mp_flow_left.end(), 0);
    
    if ((int)g_mp_agent_last_main.size() < env->num_of_agents)
        g_mp_agent_last_main.assign(env->num_of_agents + 1000, -1);
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int m = -1, b;
        project(loc, m, b);
        int prev_m = g_mp_agent_last_main[a];
        g_mp_agent_last_main[a] = m;
        if (prev_m >= 0 && m >= 0) {
            // Find nearest main point
            int best_si = 0, best_d = 999999;
            for (int si = 0; si < g_mp_count; si++) {
                int mloc = g_mp_loc[si];
                if (mloc < 0) continue;
                int mr = mloc / env->cols, mc = mloc % env->cols;
                int dr = std::abs(loc / env->cols - mr);
                int dc = std::abs(loc % env->cols - mc);
                int d = dr + dc;
                if (d < best_d) { best_d = d; best_si = si; }
            }
            if (m - prev_m >= 1)      g_mp_flow_right[best_si]++;
            else if (m - prev_m <= -1) g_mp_flow_left[best_si]++;
        }
    }
    
    // 2. Majority flow direction
    std::fill(g_mp_dir.begin(), g_mp_dir.end(), 0);
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_density[si] < 10) continue;  // DIR_THRESHOLD equivalent
        g_mp_dir[si] = (g_mp_flow_right[si] > g_mp_flow_left[si]) ? +1 :
                       (g_mp_flow_left[si] > g_mp_flow_right[si]) ? -1 : 0;
    }
    
    // Future flow direction override
    for (int si = 0; si < g_mp_count; si++) {
        float total = g_mp_future_right[si] + g_mp_future_left[si];
        if (total >= 3.0f) {
            if (g_mp_future_right[si] > g_mp_future_left[si] * 1.5f)
                g_mp_dir[si] = +1;
            else if (g_mp_future_left[si] > g_mp_future_right[si] * 1.5f)
                g_mp_dir[si] = -1;
        }
    }
    
    // 3. Direction lock: width-adaptive thresholds
    std::fill(g_mp_lock_dir.begin(), g_mp_lock_dir.end(), 0);
    
    // Phase 1: Hot zones set direction (follow neighbor if exists)
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_density[si] < g_mp_threshold[si]) continue;
        if (g_mp_dir[si] != 0) continue;
        int neighbor_dir = 0;
        for (int d = -2; d <= 2; d++) {
            int ni = si + d;
            if (ni < 0 || ni >= g_mp_count) continue;
            if (g_mp_dir[ni] != 0) { neighbor_dir = g_mp_dir[ni]; break; }
        }
        g_mp_dir[si] = (neighbor_dir != 0) ? neighbor_dir : 
                       ((si * 1103515245 + 12345) % 2 ? +1 : -1);
    }
    
        // Phase 2: Hot zones lock - density threshold OR future flow prediction
    // First pass: density >= threshold (existing)
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_density[si] < g_mp_threshold[si]) continue;
        int dir = g_mp_dir[si];
        for (int d = -2; d <= 2; d++) {
            int ni = si + d;
            if (ni < 0 || ni >= g_mp_count) continue;
            g_mp_lock_dir[ni] = dir;
        }
    }
    // Second pass: density < threshold but future flow >= 5.0 (predictive lock)
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_lock_dir[si] != 0) continue;
        float total = g_mp_future_right[si] + g_mp_future_left[si];
        if (total < 5.0f) continue;
        int dir = (g_mp_future_right[si] > g_mp_future_left[si]) ? +1 : -1;
        for (int d = -2; d <= 2; d++) {
            int ni = si + d;
            if (ni < 0 || ni >= g_mp_count) continue;
            g_mp_lock_dir[ni] = dir;
        }
    }
    
    // Phase 3: Unlock - density cooled, OR hot but no future traffic
    for (int si = 0; si < g_mp_count; si++) {
        if (g_mp_lock_dir[si] == 0) continue;
        // Rule 1: density cooled down (existing)
        if (g_mp_density[si] < g_mp_threshold[si]) {
            bool nearby_hot = false;
            for (int d = -2; d <= 2; d++) {
                int ni = si + d;
                if (ni < 0 || ni >= g_mp_count) continue;
                if (g_mp_density[ni] >= g_mp_threshold[ni]) { nearby_hot = true; break; }
            }
            if (!nearby_hot) g_mp_lock_dir[si] = 0;
            continue;
        }
        // Rule 2: still hot but no future traffic heading here (predictive unlock)
        float total = g_mp_future_right[si] + g_mp_future_left[si];
        if (total < 2.0f) {
            g_mp_lock_dir[si] = 0;
        }
    }
}
#endif
// ====== NDJSON MP analysis log (disabled for cloud) ======
// #if USE_MP_DENSITY
// static FILE* g_mp_log_f = NULL;
// static void mp_log_flush() {
//     ...
// }
// #endif

} // namespace Centerline


