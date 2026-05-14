// centerline.h - V4: Path-aligned main squares + cell-level main zone
// All density data comes from g_zone24_density (2D grid, unchanged).
// g_main_squares[] replaces g_zone_ranges[] for path-aligned statistics.
// g_cell_square[loc] replaces g_zone_is_main[zid] for cell-level main zone checks.
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

// ====== V4 Config (from config.h) ======
// SQUARE_SIZE, SQUARE_STEP, PREDICT_AHEAD, PREDICT_THRESHOLD, DIR_LOCK_RADIUS
// are defined in config.h

// ====== State (kept from V3) ======
static bool g_ready = false;
static int g_rows = 0, g_cols = 0;
static int g_total_len = 0;
static std::vector<int> g_main_dist;          // [loc] = main distance (1D)
static std::vector<int> g_branch_dist;         // [loc] = branch distance
static std::vector<int> g_centerline_cells;    // centerline path cell locations
static std::vector<int> g_zone24_density;      // 2D heatmap: [zid] = live agent count (UNCHANGED)
static int g_n24c = 0, g_n24r = 0;

// ====== V4 New: Path-aligned main squares ======
struct MainSquare {
    int cx, cy;      // center cell coords
    int min_m, max_m; // main_dist range covered by this square
    int density;      // agent count in this square (this tick)
    int lock_dir;     // direction lock: +1/-1/0
};
static std::vector<MainSquare> g_main_squares;   // sorted by path order (min_m)
static std::vector<int> g_cell_square;            // [loc] → nearest square index, -1 = not in any

// ====== REMOVED from V3 ======
// g_zone_is_main[zid]     → replaced by g_cell_square[loc]
// g_zone_ranges[]         → replaced by g_main_squares[]
// ZoneMainRange struct    → replaced by MainSquare
// g_zone_direction[zid]   → replaced by MainSquare.lock_dir
// g_zone_threshold[zid]   → removed (unused)
// g_agent_last_main[aid]  → kept for direction tracking

// ====== Direction tracking (kept from V3) ======
static std::vector<int> g_zone_direction;     // [zid] = direction (kept for compatibility)
static std::vector<int> g_agent_last_main;    // [aid] = last tick main distance

// ====== Zone-level main indicator (for connection_exempt compatibility) ======
static std::vector<int> g_zone_has_main;      // [zid] = 1 if any cell in this zone is in a main square

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

// ====== A* widest path (fallback only) ======
static void widest_path(const SharedEnvironment* env, const std::vector<int>& dtw,
                         int sx, int sy, int gx, int gy, std::vector<int>& path) {
    int total = env->rows * env->cols;
    std::vector<int> best_clr(total, -1), came_from(total, -1);
    struct Node {
        int prio, clr, steps, loc;
        bool operator>(const Node& o) const { return prio > o.prio; }
    };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    int sloc = sy * g_cols + sx;
    int sc = dtw[sloc];
    pq.push({-sc*10000, sc, 0, sloc});
    best_clr[sloc] = sc;
    int gloc = gy * g_cols + gx;
    while (!pq.empty()) {
        auto n = pq.top(); pq.pop();
        if (n.loc == gloc) break;
        if (n.clr < best_clr[n.loc]) continue;
        int r = n.loc / g_cols, c = n.loc % g_cols;
        int neigh[4] = {n.loc - g_cols, n.loc + g_cols, n.loc - 1, n.loc + 1};
        for (int nn : neigh) {
            if (nn >= 0 && nn < total && env->map[nn] == 0) {
                int nc = std::min(n.clr, dtw[nn]);
                if (nc > best_clr[nn]) {
                    best_clr[nn] = nc;
                    came_from[nn] = n.loc;
                    int nr = nn / g_cols, nc2 = nn % g_cols;
                    pq.push({-nc*10000 + n.steps+1 + std::abs(nc2-gx) + std::abs(nr-gy), nc, n.steps+1, nn});
                }
            }
        }
    }
    path.clear();
    int cur = gloc;
    while (cur != -1 && cur != sloc) { path.push_back(cur); cur = came_from[cur]; }
    if (cur == sloc) path.push_back(sloc);
    std::reverse(path.begin(), path.end());
    if (path.size() < 2) { path.clear(); path.push_back(sloc); path.push_back(gloc); }
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

// ====== Project loc -> (main, branch) - UNCHANGED from V3 ======
inline void project(int loc, int& main_dist, int& branch_dist) {
    if (!g_ready || loc < 0 || loc >= (int)g_branch_dist.size()) {
        main_dist = -1; branch_dist = -1; return;
    }
    branch_dist = g_branch_dist[loc];
    main_dist = (branch_dist >= 0) ? g_main_dist[loc] : -1;
}

// ====== Generate path-aligned squares from centerline ======
static void generate_main_squares(const SharedEnvironment* env) {
    int half = SQUARE_SIZE / 2;
    g_main_squares.clear();
    
    for (size_t i = 0; i < g_centerline_cells.size(); i += SQUARE_STEP) {
        int loc = g_centerline_cells[i];
        if (loc < 0) continue;
        int r = loc / g_cols, c = loc % g_cols;
        
        MainSquare sq;
        sq.cx = c; sq.cy = r;
        sq.min_m = INT_MAX; sq.max_m = -1;
        sq.density = 0; sq.lock_dir = 0;
        
        // Find main_dist range within this square
        for (int dr = -half; dr < half; dr++) {
            for (int dc = -half; dc < half; dc++) {
                int nr = r + dr, nc = c + dc;
                if (nr >= 0 && nr < g_rows && nc >= 0 && nc < g_cols &&
                    env->map[nr * g_cols + nc] == 0) {
                    int m = g_main_dist[nr * g_cols + nc];
                    if (m >= 0 && m < sq.min_m) sq.min_m = m;
                    if (m >= 0 && m > sq.max_m) sq.max_m = m;
                }
            }
        }
        
        g_main_squares.push_back(sq);
    }
    
    printf("[CENTERLINE V4] Generated %zu path-aligned squares (size=%d, step=%d)\n",
           g_main_squares.size(), SQUARE_SIZE, SQUARE_STEP);
}

// ====== Precompute cell → nearest square mapping ======
static void compute_cell_square_mapping(const SharedEnvironment* env) {
    int total = env->rows * env->cols;
    int half = SQUARE_SIZE / 2;
    int max_dist2 = half * half * 2;  // diagonal half-distance squared
    
    g_cell_square.assign(total, -1);
    
    for (int loc = 0; loc < total; loc++) {
        if (env->map[loc] != 0) continue;  // obstacle, skip
        
        int r = loc / g_cols, c = loc % g_cols;
        int best_d2 = INT_MAX, best_si = -1;
        
        for (int si = 0; si < (int)g_main_squares.size(); si++) {
            int dx = c - g_main_squares[si].cx;
            int dy = r - g_main_squares[si].cy;
            int d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; best_si = si; }
        }
        
        if (best_d2 <= max_dist2)
            g_cell_square[loc] = best_si;
    }
}

// ====== Init: BFS path + wall correction + main_squares + cell mapping ======
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
    
    // BFS shortest path
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
    
    std::vector<int> base_path;
    if (found_goal < 0) {
        widest_path(env, dtw, start_loc % g_cols, start_loc / g_cols,
                    goal_loc % g_cols, goal_loc / g_cols, base_path);
        smooth_path(env, base_path, base_path);
    } else {
        int cur = goal_loc;
        while (cur != start_loc) { base_path.push_back(cur); cur = came_from[cur]; }
        base_path.push_back(start_loc);
        std::reverse(base_path.begin(), base_path.end());
    }
    
    // Wall correction: for DTW <= 2 cells, scan radius 5, snap to best zone
    std::unordered_map<int, int> corrected_map;
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
    
    // V4: Generate path-aligned squares + cell→square mapping
    // (These replace V3's g_zone_is_main + g_zone_ranges + build_zone_ranges)
    generate_main_squares(env);
    compute_cell_square_mapping(env);
    
    // V4: Compute zone→has_main for connection_exempt compatibility
    g_zone_has_main.assign(g_n24c * g_n24r, 0);
    for (int loc = 0; loc < total; loc++) {
        if (g_cell_square[loc] >= 0) {
            int r = loc / g_cols, c = loc % g_cols;
            int zid = (r / 24) * g_n24c + (c / 24);
            if (zid >= 0 && zid < (int)g_zone_has_main.size())
                g_zone_has_main[zid] = 1;
        }
    }
    
    // Initialize 2D zone density (kept from V3, unchanged)
    g_zone24_density.assign(g_n24c * g_n24r, 0);
    g_zone_direction.assign(g_n24c * g_n24r, 0);
    g_agent_last_main.assign(env->num_of_agents, -1);
    
    g_ready = true;
}

// ====== 2D zone density (tick call) - UNCHANGED from V3 ======
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

// ====== Square density (V4 new: per-tick count agents per square) ======
inline void update_square_density(const SharedEnvironment* env) {
    for (auto& sq : g_main_squares)
        sq.density = 0;
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int si = g_cell_square[loc];
        if (si >= 0 && si < (int)g_main_squares.size())
            g_main_squares[si].density++;
    }
}

// ====== Zone threshold (kept for compatibility, no-op) ======
inline void update_zone_threshold() {
    // No-op: V4 uses square densities directly
}

// ====== Square direction (V4: replaces update_zone_direction) ======
inline void update_square_direction(const SharedEnvironment* env) {
    int rows = env->rows, cols = env->cols;
    
    if ((int)g_agent_last_main.size() != env->num_of_agents)
        g_agent_last_main.assign(env->num_of_agents, -1);
    
    // Reset locks
    for (auto& sq : g_main_squares)
        sq.lock_dir = 0;
    
    int curr_tick = env->curr_timestep;
    
    // Boot period: all squares locked to +1 (left→right)
    if (curr_tick < DIR_LOCK_BOOT_TICKS) {
        for (auto& sq : g_main_squares)
            sq.lock_dir = +1;
    } else {
        // Normal flow-based direction detection
        std::vector<int> flow_plus(g_main_squares.size(), 0);
        std::vector<int> flow_minus(g_main_squares.size(), 0);
        
        for (int a = 0; a < env->num_of_agents; a++) {
            int loc = env->curr_states[a].location;
            if (loc < 0) continue;
            int si = g_cell_square[loc];
            if (si < 0 || si >= (int)g_main_squares.size()) continue;
            
            int m = -1, b;
            project(loc, m, b);
            int prev_m = g_agent_last_main[a];
            g_agent_last_main[a] = m;
            
            if (prev_m >= 0 && m >= 0) {
                if (m > prev_m) flow_plus[si]++;
                else if (m < prev_m) flow_minus[si]++;
            }
        }
        
        for (size_t si = 0; si < g_main_squares.size(); si++) {
            int dir = 0;
            if (flow_plus[si] > flow_minus[si] * 3) dir = +1;
            else if (flow_minus[si] > flow_plus[si] * 3) dir = -1;
            g_main_squares[si].lock_dir = dir;
        }
    }
    
    // Propagate lock: hot squares lock self ± DIR_LOCK_RADIUS squares
    for (size_t si = 0; si < g_main_squares.size(); si++) {
        if (g_main_squares[si].density < ZONE_HARD_CAP)
            continue;
        int dir = g_main_squares[si].lock_dir;
        if (dir == 0) continue;
        
        int lo_idx = std::max(0, (int)si - DIR_LOCK_RADIUS);
        int hi_idx = std::min((int)g_main_squares.size() - 1, (int)si + DIR_LOCK_RADIUS);
        for (int sj = lo_idx; sj <= hi_idx; sj++) {
            g_main_squares[sj].lock_dir = dir;
        }
    }
    
    // Also update g_zone_direction for compatibility (from V3 zone tracking)
    // This is used by connection_exempt and other code that reads zone direction
    int n24c = (cols + 23) / 24, n24r = (rows + 23) / 24;
    int nz = n24c * n24r;
    if ((int)g_zone_direction.size() != nz)
        g_zone_direction.assign(nz, 0);
    
    for (size_t si = 0; si < g_main_squares.size() && si < PIBT_Lock::MAX_SEGMENTS; si++)
        PIBT_Lock::g_seg_lock_dir[si] = g_main_squares[si].lock_dir;
    
    // Map square lock_dir to zone lock_dir for old code compatibility
    for (int z = 0; z < nz; z++) {
        int zr = z / n24c, zc = z % n24c;
        int zcx = zc * 24 + 12, zcy = zr * 24 + 12;
        int best_d2 = INT_MAX, best_dir = 0;
        for (size_t si = 0; si < g_main_squares.size(); si++) {
            int dx = zcx - g_main_squares[si].cx;
            int dy = zcy - g_main_squares[si].cy;
            int d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; best_dir = g_main_squares[si].lock_dir; }
        }
        g_zone_direction[z] = best_dir;
    }
}

// ====== Check if a location is in main zone (V4: cell-level) ======
inline bool is_main_cell(int loc) {
    return g_ready && loc >= 0 && loc < (int)g_cell_square.size() && g_cell_square[loc] >= 0;
}

} // namespace Centerline
