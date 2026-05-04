// strategy_dist.h — BFS zone-based distance estimation + V7 scheduling
// During init(): compute BFS between zone centers → real path distances
// During plan(): use BFS(zone_agent, zone_task) + within-zone Manhattan
// This replaces the terrible Manhattan distance on 90% obstacle maps
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstdint>
#include <climits>

namespace strategy_dist {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

// Zone grid: 100x100 cells
#define ZS 100
static inline int zid(int loc, int cols) { return (loc/cols/ZS)*200 + (loc%cols/ZS); }

// BFS distance table: zone_center_id -> zone_center_id -> distance
static bool dist_initialized = false;
static std::unordered_map<int, int16_t> bfs_dist; // key = (zfrom << 16) | zto
static std::unordered_map<int, int> zone_center;  // zone_id -> nearest walkable cell

static SharedEnvironment* g_env = nullptr;
static int g_cols = 0;

static int16_t get_bfs_dist(int z1, int z2) {
    auto it = bfs_dist.find((z1 << 16) | z2);
    if (it != bfs_dist.end()) return it->second;
    it = bfs_dist.find((z2 << 16) | z1);
    if (it != bfs_dist.end()) return it->second;
    return -1;
}

void init() {
    f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear();
    bfs_dist.clear(); zone_center.clear();
    dist_initialized = false; g_env = nullptr; g_cols = 0;
}

void init_distances(SharedEnvironment* env) {
    g_env = env;
    g_cols = env->cols;
    int rows = env->rows, N = rows * g_cols;
    
    // Ensure neighbors are initialized
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    // Step 1: Find a walkable center cell for each zone
    // Scan through the map, for each zone find first walkable cell
    for (int r = 0; r < rows; r += ZS/2) { // stride by half zone for coverage
        for (int c = 0; c < g_cols; c += ZS/2) {
            int z = (r/ZS)*200 + (c/ZS);
            if (zone_center.count(z)) continue;
            // Search for nearest walkable cell to zone center
            int best_cell = -1, best_dist = 999999;
            for (int dr = 0; dr < ZS && r+dr < rows; dr++) {
                for (int dc = 0; dc < ZS && c+dc < g_cols; dc++) {
                    int cell = (r+dr)*g_cols + (c+dc);
                    if (env->map[cell] == 0) {  // walkable
                        int d = std::abs(dr - ZS/2) + std::abs(dc - ZS/2);
                        if (d < best_dist) { best_dist = d; best_cell = cell; }
                    }
                }
            }
            if (best_cell >= 0) zone_center[z] = best_cell;
        }
    }
    
    // Step 2: For each zone center, compute BFS to ALL OTHER zone centers
    // This gives us accurate cross-zone path distances
    int nzones = (int)zone_center.size();
    if (nzones == 0) { dist_initialized = true; return; }
    
    // Sort zones by their center cell ID for ordered processing
    std::vector<std::pair<int,int>> zvec(zone_center.begin(), zone_center.end());
    
    int computed = 0;
    for (auto& kv : zvec) {
        int zfrom = kv.first;
        int start = kv.second;
        
        // BFS from start
        std::queue<int> q;
        std::unordered_map<int, int16_t> dist;
        dist[start] = 0;
        q.push(start);
        
        int targets_found = 0;
        int target_count = nzones - 1;
        
        while (!q.empty() && targets_found < target_count) {
            int cur = q.front(); q.pop();
            int16_t nd = dist[cur] + 1;
            auto& ns = DefaultPlanner::global_neighbors[cur];
            for (int next : ns) {
                if (dist.find(next) != dist.end()) continue;
                dist[next] = nd;
                q.push(next);
            }
        }
        
        // Store distances to OTHER zone centers
        for (auto& kv2 : zvec) {
            int zto = kv2.first;
            if (zto == zfrom) continue;
            if (dist.find(kv2.second) != dist.end()) {
                bfs_dist[(zfrom << 16) | zto] = dist[kv2.second];
            }
        }
        
        computed++;
        if (computed >= 200) break; // Safety limit: max 200 BFS runs
    }
    
    dist_initialized = true;
}

// Estimated travel distance: agent_loc -> task at tid
static inline float get_travel(int a_loc, int tid, SharedEnvironment* env) {
    int cols = env->cols;
    int t_loc = env->task_pool[tid].locations[0];
    int az = zid(a_loc, cols);
    int tz = zid(t_loc, cols);
    
    if (az == tz) {
        // Same zone: use Manhattan (zone is only 100x100, obstacles within are limited)
        int ar = a_loc / cols, ac = a_loc % cols;
        int tr = t_loc / cols, tc = t_loc % cols;
        return (float)(std::abs(ar - tr) + std::abs(ac - tc));
    }
    
    // Different zone: use BFS zone-to-zone distances
    int16_t z2z = get_bfs_dist(az, tz);
    if (z2z > 0) {
        return (float)z2z;
    }
    
    // Fallback: Manhattan with obstacle correction
    int ar = a_loc / cols, ac = a_loc % cols;
    int tr = t_loc / cols, tc = t_loc % cols;
    int man = std::abs(ar - tr) + std::abs(ac - tc);
    return (float)man * 3.0f;  // Rough correction for 90% obstacles
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;

    // Score new tasks
    for (int tid : env->new_tasks) {
        if (!tbirth.count(tid)) tbirth[tid] = step;
        float idist = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            idist += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (idist > 0) ? (float)n/idist : 10.0f;
        tval[tid] = (n == 1) ? base * 1.5f : base;
    }

    f_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    f_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (int a = 0; a < env->num_of_agents; a++)
        if (env->curr_task_schedule[a] < 0) f_agents.insert(a);
    if (f_agents.empty() || f_tasks.empty()) {
        for (int a : f_agents) proposed_schedule[a] = -1;
        return;
    }

    // Zone-interleaved ordering
    std::unordered_map<int, std::vector<int>> az;
    for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
    std::vector<int> agents;
    for (bool more = true; more; ) {
        more = false;
        for (auto& kv : az)
            if (!kv.second.empty()) { agents.push_back(kv.second.back()); kv.second.pop_back(); more = true; }
    }

    // Assign with corrected distances
    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (!f_agents.count(a)) continue;
        int a_loc = env->curr_states.at(a).location;
        int a_zone = zid(a_loc, cols);
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) && env->task_pool[cur].locations.size() > 1 && f_tasks.count(cur)) {
            proposed_schedule[a] = cur; f_agents.erase(a); f_tasks.erase(cur); continue;
        }
        int best = -1;
        float best_score = -1e9f;
        for (int tid : f_tasks) {
            float travel = get_travel(a_loc, tid, env);
            int t_zone = zid(env->task_pool[tid].locations[0], cols);
            float zbonus = (t_zone == a_zone) ? 1.5f : 1.0f; // Mild locality (BFS already accounts for path cost)
            int birth = tbirth.count(tid) ? tbirth[tid] : step;
            int wait = step - birth;
            float age = 1.0f + (float)wait * 0.02f;
            if (wait > 60) age *= (1.0f + (float)(wait - 60) * 0.05f);
            float score = tval[tid] * zbonus * age / (travel + 1.0f);
            if (score > best_score) { best_score = score; best = tid; }
        }
        if (best >= 0) {
            proposed_schedule[a] = best;
            f_agents.erase(a); f_tasks.erase(best);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_dist
