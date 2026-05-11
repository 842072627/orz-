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

static constexpr int CONGESTION_THRESHOLD = 40;
static constexpr int DEADLOCK_THRESHOLD = 90;
static constexpr float MAX_PENALTY = 100.0f;
// ====== Adaptive per-zone threshold (dynamic, replaces HOT_BLOCK + CONTAIN_LOCK) ======
// V4: per-zone capacity-aware threshold
// Formula: target = floor + min(0.4*density, (capacity - floor)*0.8)
// Higher density -> higher threshold (more permissive), but capped by zone capacity
// Capacity = walkable cells in zone * 0.5 (50% occupancy max)
static constexpr float ADAPTIVE_ALPHA = 0.1f;
static constexpr int ADAPTIVE_FLOOR = 12;  // Floor: minimum threshold (was 8)
static constexpr int ADAPTIVE_CEIL = 50;   // V14 ceiling
// Expansion for direction lock neighbors
static constexpr int CONTAIN_EXPAND = 1;
static constexpr int HOT_LOCK_COOLDOWN = 30;
static constexpr int DIR_THRESHOLD = 10;

// ====== State ======
static bool g_ready = false;
static int g_rows = 0, g_cols = 0;
static int g_total_len = 0;
static std::vector<int> g_main_dist;        // [loc] = main distance
static std::vector<int> g_branch_dist;      // [loc] = branch distance
static std::vector<int> g_centerline_cells;  // centerline cell locations
static std::vector<int> g_zone24_density;   // 2D heatmap: [zid] = live agent count
static std::vector<int> g_zone_threshold;    // [zid] = adaptive threshold (12-80)
static std::vector<int> g_zone_capacity;     // [zid] = max safe capacity (based on corridor DTW width)
static std::vector<int> g_zone_walkable;     // [zid] = walkable cells in zone
static std::vector<int> g_zone_slope;        // [zid] = density change rate (per tick)
static std::vector<int> g_zone_prev_density; // [zid] = last tick density
static std::vector<int> g_zone_corridor_dtw; // [zid] = avg distance-to-wall of main corridor cells (road width proxy)
static int g_n24c = 0, g_n24r = 0;
static std::vector<int> g_zone_is_main;     // [zid] = 1 if centerline passes through

// ====== Zoneâ†’main range (replaces segment system) ======
struct ZoneMainRange {
    int zone_id;      // 24Ã—24 zone id
    int min_m, max_m; // centerline cells' main_dist range in this zone
    int lock_dir;     // direction lock: +1/-1/0
};
static std::vector<ZoneMainRange> g_zone_ranges;

// ====== Direction tracking ======
static std::vector<int> g_zone_direction;    // [+1/-1/0] per zone
static std::vector<int> g_agent_last_main;   // [aid] = last tick main distance

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
        int lock = (zid >= 0 && zid < (int)g_zone_direction.size()) ? g_zone_direction[zid] : 0;
        g_zone_ranges.push_back({zid, kv.second.first, kv.second.second, lock});
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
    g_zone_threshold.assign(g_n24c * g_n24r, 23);
    
    // Mark main-line zones (must be BEFORE capacity estimation which uses g_zone_is_main)
    g_zone_is_main.assign(g_n24c * g_n24r, 0);
    for (int clloc : g_centerline_cells) {
        int r = clloc / g_cols, c = clloc % g_cols;
        int zid = (r / 24) * g_n24c + (c / 24);
        if (zid >= 0 && zid < (int)g_zone_is_main.size())
            g_zone_is_main[zid] = 1;
    }
    
    // Compute per-zone walkable cell count
    g_zone_walkable.assign(g_n24c * g_n24r, 0);
    for (int i = 0; i < total; i++) {
        if (env->map[i] == 0) {
            int r = i / g_cols, c = i % g_cols;
            int zid = (r / 24) * g_n24c + (c / 24);
            if (zid >= 0 && zid < (int)g_zone_walkable.size())
                g_zone_walkable[zid]++;
        }
    }
    
    // Capacity = 50% of walkable cells (simple V4 approach), clamp [15, 120]
    g_zone_capacity.assign(g_n24c * g_n24r, 20);
    for (int zi = 0; zi < g_n24c * g_n24r; zi++) {
        int cap = g_zone_walkable[zi] * 5 / 10;
        if (cap < 15) cap = 15;
        if (cap > 120) cap = 120;
        g_zone_capacity[zi] = cap;
    }
    
    // Initialize slope tracking
    g_zone_slope.assign(g_n24c * g_n24r, 0);
    g_zone_prev_density.assign(g_n24c * g_n24r, 0);
    
    
    build_zone_ranges();
    g_ready = true;
}

// ====== 2D zone density (tick call) + slope tracking ======
inline void update_zone24_density(const SharedEnvironment* env) {
    int n24c = (env->cols + 23) / 24, n24r = (env->rows + 23) / 24;
    // Save previous density for slope calculation
    if ((int)g_zone_prev_density.size() != n24c * n24r)
        g_zone_prev_density.assign(n24c * n24r, 0);
    else
        std::copy(g_zone24_density.begin(), g_zone24_density.end(), g_zone_prev_density.begin());
    
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
    
    // Slope = per-tick density change (smoothed with exponential moving average)
    if ((int)g_zone_slope.size() != n24c * n24r)
        g_zone_slope.assign(n24c * n24r, 0);
    for (int zi = 0; zi < n24c * n24r; zi++) {
        int delta = g_zone24_density[zi] - g_zone_prev_density[zi];
        // EMA smoothing: 30% current + 70% previous
        g_zone_slope[zi] = (g_zone_slope[zi] * 7 + delta * 3) / 10;
    }
}

// ====== V13: V3 inverse formula = density UP → threshold DOWN ======
// target = 35 - density*0.6, clamp(8, 35), hard block when d >= thr
// density 0→target 35→thr~35 (permissive), density 20→target 23→thr~23 (limiting)
// density 25→target 20→thr~20→HARD BLOCK d=25>=thr=20
// density 30→target 17→thr~17→HARD blocked
// V18: V14 inverse formula + hard block = self-limiting dynamic zones
static constexpr float F0_INV = -0.5f;
static constexpr float INV_CEIL = 50.0f;
static constexpr float V13_ALPHA = 0.15f;
inline void update_zone_threshold() {
    int nz = (int)g_zone24_density.size();
    if ((int)g_zone_threshold.size() != nz) g_zone_threshold.assign(nz, 23);
    float oma = 1.0f - V13_ALPHA;
    for (int zid = 0; zid < nz; zid++) {
        float target = INV_CEIL + (float)g_zone24_density[zid] * F0_INV;
        int new_thr = (int)((float)g_zone_threshold[zid] * oma + target * V13_ALPHA + 0.5f);
        if (new_thr < ADAPTIVE_FLOOR) new_thr = ADAPTIVE_FLOOR;
        if (new_thr > ADAPTIVE_CEIL) new_thr = ADAPTIVE_CEIL;
        g_zone_threshold[zid] = new_thr;
    }
}

// ====== Zone direction (tick call) ======
inline void update_zone_direction(const SharedEnvironment* env) {
    int rows = env->rows, cols = env->cols;
    int n24c = (cols + 23) / 24, n24r = (rows + 23) / 24;
    int nz = n24c * n24r;
    if ((int)g_zone_direction.size() != nz) g_zone_direction.assign(nz, 0);
    if ((int)g_agent_last_main.size() != env->num_of_agents) g_agent_last_main.assign(env->num_of_agents, -1);
    
    std::vector<int> flow_right(nz, 0), flow_left(nz, 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int r = loc / cols, c = loc % cols;
        int zi = (r / 24) * n24c + (c / 24);
        if (zi < 0 || zi >= nz) continue;
        int m = -1, b;
        project(loc, m, b);
        int prev_m = g_agent_last_main[a];
        g_agent_last_main[a] = m;
        if (prev_m >= 0 && m >= 0) {
            if (m - prev_m >= 1)      flow_right[zi]++;
            else if (m - prev_m <= -1) flow_left[zi]++;
        }
    }
    
    // Zone direction: majority flow
    std::fill(g_zone_direction.begin(), g_zone_direction.end(), 0);
    for (int zi = 0; zi < nz; zi++) {
        if (g_zone24_density[zi] < DIR_THRESHOLD) continue;
        g_zone_direction[zi] = (flow_right[zi] > flow_left[zi]) ? +1 :
                               (flow_left[zi] > flow_right[zi]) ? -1 : 0;
    }
    
    if (ORZ_DIRECTION_LOCK) {
        // Non-merge direction lock: each hot zone locks itself \u00b1CONTAIN_EXPAND neighbors independently
        // No block merging - each zone unlocks when its neighborhood cools down
        
        // Phase 1: Hot zones set direction (follow neighbor direction if exists)
        for (int i = 0; i < (int)g_zone_ranges.size(); i++) {
            int zid = g_zone_ranges[i].zone_id;
            if (zid < 0 || zid >= nz || g_zone24_density[zid] < g_zone_threshold[zid])
                continue;
            if (g_zone_direction[zid] != 0)
                continue; // Already has direction
            
            // Look for neighbor direction (first come first serve)
            int neighbor_dir = 0;
            for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
                int ni = i + d;
                if (ni < 0 || ni >= (int)g_zone_ranges.size()) continue;
                int nzid = g_zone_ranges[ni].zone_id;
                if (nzid >= 0 && g_zone_direction[nzid] != 0) {
                    neighbor_dir = g_zone_direction[nzid];
                    break;
                }
            }
            
            if (neighbor_dir != 0) {
                g_zone_direction[zid] = neighbor_dir;
            } else {
                // Use own flow direction
                int dir = (g_zone_direction[zid] != 0) ? g_zone_direction[zid] : 0;
                if (dir == 0)
                    dir = (zid * 1103515245 + 12345) % 2 ? +1 : -1;
                g_zone_direction[zid] = dir;
            }
        }
        
        // V16: Phase 2 removed - no unconditional propagation (causes direction conflicts)
        // V16: Phase 3 removed - each hot zone keeps its own direction from flow data
        // Each zone decides its own direction based on traffic, no cascading
        for (int i = 0; i < (int)g_zone_ranges.size(); i++) {
            int zid = g_zone_ranges[i].zone_id;
            if (zid < 0 || zid >= nz) continue;
            if (g_zone24_density[zid] >= g_zone_threshold[zid])
                continue; // Hot zone stays locked
            
            bool nearby_hot = false;
            for (int d = -CONTAIN_EXPAND; d <= CONTAIN_EXPAND; d++) {
                int ni = i + d;
                if (ni < 0 || ni >= (int)g_zone_ranges.size()) continue;
                int nzid = g_zone_ranges[ni].zone_id;
                if (nzid >= 0 && g_zone24_density[nzid] >= g_zone_threshold[nzid]) {
                    nearby_hot = true;
                    break;
                }
            }
            if (!nearby_hot)
                g_zone_direction[zid] = 0; // Unlock
        }
    }
    // Sync zone direction to zone_ranges
    for (auto& zr : g_zone_ranges) {
        if (zr.zone_id >= 0 && zr.zone_id < nz)
            zr.lock_dir = ORZ_DIRECTION_LOCK ? g_zone_direction[zr.zone_id] : 0;
    }
    
    // Sync to PIBT lock
    for (size_t si = 0; si < g_zone_ranges.size() && si < (size_t)PIBT_Lock::MAX_SEGMENTS; si++)
        PIBT_Lock::g_seg_lock_dir[si] = g_zone_ranges[si].lock_dir;
}

} // namespace Centerline


