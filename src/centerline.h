// centerline.h - ORZ centerline generation + BFS distance field projection
//
// Precomputation (init once):
//   1. BFS distance-to-wall from all obstacle cells
//   2. A* widest path: leftmost -> rightmost walkable cells
//   3. Moving average smoothing + snap to walkable
//   4. BFS from centerline cells -> branch_dist[]
//   5. main_dist[] = cumulative distance along centerline
//
// Runtime:
//   project(loc) -> (main_dist, branch_dist)  O(1)
//   update_zone_density(env)                  O(N), call each tick
//   get_congestion_factor(task_id, env)       O(Z), Z=zones on path
//
// Thresholds from heatmap analysis (R453, 1800 agents, last 300 ticks):
//   CONGESTION_THRESHOLD = 44 (595,428 level, 24x24(576-599,408-431) = 13165/300)
//   DEADLOCK_THRESHOLD   = 90 (635,491 level, 24x24(624-647,480-503) = 26865/300)

#pragma once
#include "SharedEnv.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <cstring>
#include <climits>
#include <cstdio>
#include <map>

namespace Centerline {

// ====== Config (heatmap-calibrated, can be adapted per tick) ======
static constexpr int CONGESTION_THRESHOLD = 40;   // agents/zone: start penalty
static constexpr int DEADLOCK_THRESHOLD = 90;      // agents/zone: max penalty
static constexpr float MAX_PENALTY = 100.0f;        // score / 3 at deadlock

// ====== State ======
static bool g_ready = false;
static int g_rows = 0, g_cols = 0;
static int g_total_len = 0;
static std::vector<int> g_main_dist;       // [loc] = main distance
static std::vector<int> g_branch_dist;     // [loc] = branch distance
static std::vector<int> g_centerline_cells; // list of centerline cell locations
static std::vector<int> g_zone24_density;  // [row*n24c+col] live agent count
static int g_n24c = 0, g_n24r = 0;

// ====== 1D main-line segments (agent→task 穿越惩罚) ======
struct Segment {
    float left_m, right_m;   // 70% 重叠边界
    int zone_id;              // 对应的 2D grid zone
};
static std::vector<Segment> g_segments;
static std::vector<int> g_main_to_seg;  // [main_dist] = segment index, O(1) lookup

// ====== 7-segment direction lock state ======
static std::vector<int> g_seg_hot;         // [seg_idx] = 1 hot, 0 cool
static std::vector<int> g_seg_lock_dir;     // [seg_idx] = +1/-1
static std::vector<int> g_seg_cooldown;     // [seg_idx] = remaining ticks

// ====== Hot zone direction lock state (declared before init()) ======
static constexpr bool HOT_LOCK_ENABLED = true;
static constexpr int HOT_LOCK_DENSITY = 28;
static constexpr int HOT_LOCK_COOLDOWN = 30;
static std::vector<int> g_zone_is_main;

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
        int r = cur / g_cols, c = cur % g_cols;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total && env->map[n] == 0 && nd < dtw[n])
                { dtw[n] = nd; q.push(n); }
    }
}

// ====== A* widest path ======
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

// ====== Smooth path ======
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

// Forward declarations for build_segments / m_to_segment (defined after init)
inline void build_segments();
inline int m_to_segment(int m);

// ====== Init ======
void init(const SharedEnvironment* env) {
    g_rows = env->rows; g_cols = env->cols;
    g_n24c = env->cols / 24 + 1; g_n24r = env->rows / 24 + 1;
    int total = g_rows * g_cols;

    // Distance to wall
    std::vector<int> dtw;
    compute_dtw(env, dtw);

    // Leftmost / rightmost cells
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
    if (start_loc < 0 || goal_loc < 0) { fprintf(stderr, "Centerline: no walkable cells\n"); return; }

    // Widest path A*
    std::vector<int> raw;
    widest_path(env, dtw, start_loc % g_cols, start_loc / g_cols,
                goal_loc % g_cols, goal_loc / g_cols, raw);

    // Smooth
    std::vector<int> smooth;
    smooth_path(env, raw, smooth);
    g_centerline_cells = smooth;

    // Main distances along centerline (compute first, BFS needs them)
    std::unordered_map<int, int> cl_main; // loc -> cumulative
    int cumul = 0;
    cl_main[smooth[0]] = cumul;
    for (size_t i = 1; i < smooth.size(); i++) {
        int r1 = smooth[i-1]/g_cols, c1 = smooth[i-1]%g_cols;
        int r2 = smooth[i]/g_cols, c2 = smooth[i]%g_cols;
        cumul += (int)std::round(std::sqrt((c2-c1)*(c2-c1) + (r2-r1)*(r2-r1)));
        cl_main[smooth[i]] = cumul;
    }
    g_total_len = cumul;

    // Multi-source BFS from centerline -> branch_dist + nearest main_dist
    // BFS guarantees first visit = nearest centerline cell, so main_dist propagates directly
    // Interpolate extra seeds between centerline cells for finer main distance precision
    g_branch_dist.assign(total, -1);
    g_main_dist.assign(total, -1);
    std::queue<int> q;
    const int SUB_SAMPLES = 20;  // higher sampling density for smoother centerline
    for (size_t i = 0; i < smooth.size(); i++) {
        int loc = smooth[i];
        g_branch_dist[loc] = 0;
        g_main_dist[loc] = cl_main[loc];
        q.push(loc);
        
        // Add interpolated seeds between this cell and the next
        if (i + 1 < smooth.size()) {
            int nloc = smooth[i+1];
            int r1 = loc / g_cols, c1 = loc % g_cols;
            int r2 = nloc / g_cols, c2 = nloc % g_cols;
            int m1 = cl_main[loc], m2 = cl_main[nloc];
            for (int k = 1; k < SUB_SAMPLES; k++) {
                float t = (float)k / (float)SUB_SAMPLES;
                int ir = (int)std::round(r1 + (r2 - r1) * t);
                int ic = (int)std::round(c1 + (c2 - c1) * t);
                int iloc = ir * g_cols + ic;
                if (iloc >= 0 && iloc < total && env->map[iloc] == 0 && g_branch_dist[iloc] == -1) {
                    g_branch_dist[iloc] = 0;
                    g_main_dist[iloc] = (int)std::round(m1 + (m2 - m1) * t);
                    q.push(iloc);
                }
            }
        }
    }
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int nd = g_branch_dist[cur] + 1;
        int nmain = g_main_dist[cur];
        int r = cur / g_cols, c = cur % g_cols;
        int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
        for (int n : neigh)
            if (n >= 0 && n < total && env->map[n] == 0 && g_branch_dist[n] == -1)
                { g_branch_dist[n] = nd; g_main_dist[n] = nmain; q.push(n); }
    }

    // Zone density init
    g_zone24_density.assign(g_n24c * g_n24r, 0);

    printf("Centerline: %zu cells, total_len=%d, zones=%dx%d (%d)\n",
           smooth.size(), g_total_len, g_n24c, g_n24r, g_n24c*g_n24r);
    
    // Mark main-line zones (centerline passes through)
    g_zone_is_main.assign(g_n24c * g_n24r, 0);
    for (int clloc : smooth) {
        int r = clloc / g_cols, c = clloc % g_cols;
        int zid = (r / 24) * g_n24c + (c / 24);
        if (zid >= 0 && zid < (int)g_zone_is_main.size())
            g_zone_is_main[zid] = 1;
    }
    
    g_ready = true;
    
    // ====== Verification: project test points and print ======
    {
        int cols = g_cols;
        struct TestPt { int x, y; };
        TestPt pts[] = {{993,308},{1143,310},{468,370},{635,493}};
        for (size_t pi = 0; pi < 4; pi++) {
            int loc = pts[pi].y * cols + pts[pi].x;
            int m, b;
            project(loc, m, b);
            int nx = -1, ny = -1;
            int best_d = 999999;
            for (size_t j = 0; j < smooth.size(); j++) {
                int cr = smooth[j]/cols, cc = smooth[j]%cols;
                int d = std::abs(pts[pi].y-cr) + std::abs(pts[pi].x-cc);
                if (d < best_d) { best_d = d; nx = cc; ny = cr; }
            }
            float pct = (g_total_len > 0) ? (float)m * 100.0f / (float)g_total_len : 0.0f;
            printf("CLVERIFY (%d,%d): M=%d (%.1f%%), B=%d, nearest=(%d,%d)\n",
                   pts[pi].x, pts[pi].y, m, pct, b, nx, ny);
        }
    }
    
    build_segments();
    g_ready = true;
}

// ====== 1D main-line segments init ======
inline void build_segments() {
    std::map<int, std::pair<int,int>> zone_m_range;
    for (int loc : g_centerline_cells) {
        if (loc < 0) continue;
        int m = g_main_dist[loc];
        int r = loc / g_cols, c = loc % g_cols;
        int zid = (r / 24) * g_n24c + (c / 24);
        auto& p = zone_m_range[zid];
        if (p.first == 0 || m < p.first) p.first = m;
        if (m > p.second) p.second = m;
    }

    std::vector<int> sorted_zids;
    for (auto& kv : zone_m_range) sorted_zids.push_back(kv.first);
    std::sort(sorted_zids.begin(), sorted_zids.end(),
        [&](int a, int b) { return zone_m_range[a].first < zone_m_range[b].first; });

    struct ZoneInfo { int zid; float center; };
    std::vector<ZoneInfo> zones;
    for (int zid : sorted_zids) {
        auto& r = zone_m_range[zid];
        zones.push_back({zid, (float)(r.first + r.second) / 2.0f});
    }

    g_segments.clear();
    g_segments.reserve(zones.size());
    for (size_t i = 0; i < zones.size(); i++) {
        float c = zones[i].center;
        float left = (i == 0) ? 0.0f : c - 0.7f * (c - zones[i-1].center);
        float right = (i == zones.size() - 1) ? (float)g_total_len + 0.001f : c + 0.7f * (zones[i+1].center - c);
        g_segments.push_back({left, right, zones[i].zid});
    }

    g_main_to_seg.assign(g_total_len + 2, -1);
    for (int m = 0; m <= g_total_len; m++) {
        float mf = (float)m;
        for (size_t i = 0; i < g_segments.size(); i++) {
            if (mf >= g_segments[i].left_m && mf < g_segments[i].right_m) {
                g_main_to_seg[m] = (int)i;
                break;
            }
        }
    }

    g_seg_hot.assign(g_segments.size(), 0);
    g_seg_lock_dir.assign(g_segments.size(), 0);
    g_seg_cooldown.assign(g_segments.size(), 0);

    printf("SEGMENTS: %zu built from %zu zones\n", g_segments.size(), zones.size());
}

inline int m_to_segment(int m) {
    if (m < 0 || m > g_total_len) return -1;
    if (g_main_to_seg.empty()) return -1;
    if (m >= (int)g_main_to_seg.size()) return -1;
    return g_main_to_seg[m];
}

// ====== 1D main-line congestion density (agent per main distance unit) ======
// Each tick: project agents to main distance, build histogram
// Task congestion: read density at task's main position
static std::vector<int> g_main_density;  // [main_dist] = live agent count

// Update 1D main-line density (call each tick)
inline void update_main_density(const SharedEnvironment* env) {
    if ((int)g_main_density.size() != g_total_len + 1)
        g_main_density.assign(g_total_len + 1, 0);
    else
        std::fill(g_main_density.begin(), g_main_density.end(), 0);
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int m, b;
        project(loc, m, b);
        if (m >= 0 && m <= g_total_len) {
            // Weight agents on/branch corridor more heavily
            float w = 1.0f / (1.0f + (float)b * 0.05f);  // branch=0→1.0, branch=10→0.67, branch=50→0.29
            // Distribute weight to adjacent bins for smoothness
            int idx = m;
            if (idx >= 0 && idx <= g_total_len) {
                // Simple: just add to nearest bin (fast)
                g_main_density[idx] += (int)(w + 0.5f);
            }
        }
    }
}

// ====== Sliding window smoothed density (O(g_total_len) per tick) ======
// Precomputed after update_main_density for O(1) range queries
static std::vector<float> g_main_smoothed;

inline void smooth_main_density() {
    if ((int)g_main_smoothed.size() != g_total_len + 1)
        g_main_smoothed.assign(g_total_len + 1, 0.0f);
    
    int half_w = 24;
    // Sliding window sum
    float sum = 0.0f;
    int cnt = 0;
    for (int i = 0; i <= half_w && i <= g_total_len; i++) {
        sum += (float)g_main_density[i];
        cnt++;
    }
    g_main_smoothed[0] = sum / (float)cnt;
    
    for (int m = 1; m <= g_total_len; m++) {
        int lo = m - half_w - 1;
        int hi = m + half_w;
        if (lo >= 0) { sum -= (float)g_main_density[lo]; cnt--; }
        if (hi <= g_total_len) { sum += (float)g_main_density[hi]; cnt++; }
        g_main_smoothed[m] = sum / (float)cnt;
    }
}

// ====== 1D task congestion cache with cached factor ======
// Stores main range + cached congestion factor (refreshed each tick)
struct Task1DCache {
    int m_start, m_end;
    float cached_factor;  // refreshed each tick
    bool valid;
};
static std::unordered_map<int, Task1DCache> g_task1d_cache;

// Precompute main range for a task (one-time, at task reveal)
inline void precompute_task_1d(int task_id, const SharedEnvironment* env) {
    if (g_task1d_cache.count(task_id)) return;
    auto it = env->task_pool.find(task_id);
    if (it == env->task_pool.end() || it->second.locations.size() < 1) return;
    
    const auto& locs = it->second.locations;
    Task1DCache cache;
    cache.valid = false;
    cache.cached_factor = 1.0f;
    cache.m_start = INT_MAX/2;
    cache.m_end = 0;
    
    for (size_t e = 0; e < locs.size(); e++) {
        int m, b;
        project(locs[e], m, b);
        if (m >= 0) {
            if (m < cache.m_start) cache.m_start = m;
            if (m > cache.m_end) cache.m_end = m;
            cache.valid = true;
        }
    }
    
    if (cache.valid) {
        cache.m_start = std::max(0, cache.m_start - 48);
        cache.m_end = std::min(g_total_len, cache.m_end + 48);
        g_task1d_cache[task_id] = cache;
    }
}

// Refresh congestion factors for all cached tasks (O(N_tasks × range))
// Call once per tick after update_main_density + smooth_main_density
inline void refresh_task_1d_congestion() {
    for (auto& kv : g_task1d_cache) {
        Task1DCache& c = kv.second;
        if (!c.valid || c.m_start > c.m_end) {
            c.cached_factor = 1.0f;
            continue;
        }
        // Find max smoothed density in range (no inner window loop!)
        int worst = 0;
        for (int m = c.m_start; m <= c.m_end; m++) {
            int d = (int)(g_main_smoothed[m] + 0.5f);
            if (d > worst) worst = d;
        }
        
        if (worst < CONGESTION_THRESHOLD) {
            c.cached_factor = 1.0f;
        } else if (worst >= DEADLOCK_THRESHOLD) {
            c.cached_factor = MAX_PENALTY;
        } else {
            float ratio = (float)(worst - CONGESTION_THRESHOLD) / (float)(DEADLOCK_THRESHOLD - CONGESTION_THRESHOLD);
            c.cached_factor = 1.0f + ratio * (MAX_PENALTY - 1.0f);
        }
    }
}

// Get cached congestion factor for a task (O(1))
inline float get_task_1d_congestion(int task_id) {
    auto it = g_task1d_cache.find(task_id);
    if (it == g_task1d_cache.end()) return 1.0f;
    return it->second.cached_factor;
}

// ====== Backward compatibility aliases for strategy_tidal ======
inline void update_zone_density(const SharedEnvironment* env) { update_main_density(env); }
inline void precompute_task_zones(int task_id, const SharedEnvironment* env) { precompute_task_1d(task_id, env); }
inline void refresh_congestion(const SharedEnvironment*) { }
inline float get_task_congestion(int task_id) { return get_task_1d_congestion(task_id); }


// ====== Zone-based direction tracking (V3) ======
static constexpr int DIR_THRESHOLD = 10;

static std::vector<int> g_zone_direction;               // [+1/-1/0] per zone (may be overridden by lock)
static std::vector<int> g_agent_last_main;           // [aid] = last tick main distance



// Tick call: compute agent count per 24x24 zone
inline void update_zone24_density(const SharedEnvironment* env) {
    int rows = env->rows, cols = env->cols;
    int n24c = (cols + 23) / 24, n24r = (rows + 23) / 24;
    if ((int)g_zone24_density.size() != n24c * n24r)
        g_zone24_density.assign(n24c * n24r, 0);
    else
        std::fill(g_zone24_density.begin(), g_zone24_density.end(), 0);
    for (int a = 0; a < env->num_of_agents; a++) {
        int loc = env->curr_states[a].location;
        if (loc < 0) continue;
        int r = loc / cols, c = loc % cols;
        int zi = (r / 24) * n24c + (c / 24);
        if (zi >= 0 && zi < (int)g_zone24_density.size())
            g_zone24_density[zi]++;
    }
}

// Tick call: compute per-zone traffic direction
inline void update_zone_direction(const SharedEnvironment* env) {
    int rows = env->rows, cols = env->cols;
    int n24c = (cols + 23) / 24, n24r = (rows + 23) / 24;
    int nz = n24c * n24r;
    if ((int)g_zone_direction.size() != nz)
        g_zone_direction.assign(nz, 0);
    if ((int)g_agent_last_main.size() != env->num_of_agents)
        g_agent_last_main.assign(env->num_of_agents, -1);
    
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
            int diff = m - prev_m;
            if (diff >= 1)      flow_right[zi]++;
            else if (diff <= -1) flow_left[zi]++;
        }
    }
    
    std::fill(g_zone_direction.begin(), g_zone_direction.end(), 0);
    for (int zi = 0; zi < nz; zi++) {
        if (g_zone24_density[zi] < DIR_THRESHOLD) continue;
        if (flow_right[zi] > flow_left[zi])
            g_zone_direction[zi] = +1;
        else if (flow_left[zi] > flow_right[zi])
            g_zone_direction[zi] = -1;
    }
    
    // ====== Hot zone direction locking: density >= HOT_LOCK_DENSITY → lock immediately ======
    if (HOT_LOCK_ENABLED) {
        // === Collect hot segments ===
        std::vector<int> hot_segs;
        for (size_t si = 0; si < g_segments.size(); si++) {
            int zid = g_segments[si].zone_id;
            if (zid >= 0 && zid < (int)g_zone24_density.size() &&
                g_zone24_density[zid] >= HOT_LOCK_DENSITY)
                hot_segs.push_back((int)si);
        }

        // === Merge overlapping 7-segment ranges ===
        for (size_t i = 0; i < hot_segs.size(); ) {
            int lo = std::max(0, hot_segs[i] - 3);
            int hi = std::min((int)g_segments.size() - 1, hot_segs[i] + 3);

            size_t j = i + 1;
            while (j < hot_segs.size() && hot_segs[j] - 3 <= hi) {
                hi = std::max(hi, std::min((int)g_segments.size() - 1, hot_segs[j] + 3));
                j++;
            }

            // Direction: pick the direction of the densest segment in the block
            int best_dir = +1, max_d = -1;
            for (size_t k = i; k < j; k++) {
                int zid2 = g_segments[hot_segs[k]].zone_id;
                int d2 = g_zone24_density[zid2];
                if (d2 > max_d) { max_d = d2; best_dir = g_zone_direction[zid2]; if (!best_dir) best_dir = +1; }
            }

            // Lock all segments in the merged range
            for (int si2 = lo; si2 <= hi; si2++) {
                int zid2 = g_segments[si2].zone_id;
                g_zone_direction[zid2] = best_dir;
                g_seg_hot[si2] = 1;
                g_seg_lock_dir[si2] = best_dir;
                g_seg_cooldown[si2] = HOT_LOCK_COOLDOWN;
            }

            i = j;
        }

        // === Cooldown for segments no longer hot ===
        for (size_t si = 0; si < g_segments.size(); si++) {
            if (!g_seg_hot[si]) continue;
            int zid2 = g_segments[si].zone_id;
            int d2 = (zid2 >= 0 && zid2 < (int)g_zone24_density.size()) ? g_zone24_density[zid2] : 0;
            if (d2 >= HOT_LOCK_DENSITY) {
                g_seg_cooldown[si] = HOT_LOCK_COOLDOWN;
            } else if (--g_seg_cooldown[si] <= 0) {
                g_seg_hot[si] = 0;
            } else {
                g_zone_direction[zid2] = g_seg_lock_dir[si];
            }
        }
    }
}

} // namespace Centerline
