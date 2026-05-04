// strategy_zonebfs.h — Zone-level BFS: real path distances precomputed during init() (6s)
// Uses 124 zone centers (50x50 cells) for accurate cross-zone distance estimation
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstdint>

namespace strategy_zonebfs {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

#define ZS 50  // 50x50 zones → ~30×14 = ~420 zones
static inline int zid(int loc, int cols) { return (loc/cols/ZS)*500 + (loc%cols/ZS); }

static bool inited = false;
static std::unordered_map<int, int16_t> bfs_dist; // (zfrom<<16)|zto -> bfs distance
static std::unordered_map<int, int> zone_cell;     // zone -> nearest walkable cell

void init() {
    f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear();
    bfs_dist.clear(); zone_cell.clear(); inited = false;
}

void init_distances(SharedEnvironment* env) {
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    int cols = env->cols, rows = env->rows;
    
    // Find a walkable cell per zone
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int loc = r * cols + c;
            if (env->map[loc] == 0) {  // Walkable
                int z = (r/ZS)*500 + (c/ZS);
                if (!zone_cell.count(z)) zone_cell[z] = loc;
            }
        }
    }
    
    // Collect zone IDs, BFS between zone centers
    std::vector<int> zids;
    for (auto& kv : zone_cell) zids.push_back(kv.first);
    
    if (zids.empty()) { inited = true; return; }
    
    // Reusable dist array
    int N = rows * cols;
    std::vector<int16_t> dist(N, -1);
    std::vector<int> visited;
    visited.reserve(100000);
    
    for (int z_from : zids) {
        int start = zone_cell[z_from];
        if (start < 0) continue;
        
        // BFS
        std::queue<int> q;
        q.push(start); dist[start] = 0; visited.push_back(start);
        
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int16_t nd = dist[cur] + 1;
            for (int nb : DefaultPlanner::global_neighbors[cur]) {
                if (dist[nb] >= 0) continue;
                dist[nb] = nd;
                visited.push_back(nb);
                q.push(nb);
            }
        }
        
        // Extract distances to other zone centers
        for (int z_to : zids) {
            if (z_to == z_from) continue;
            int tgt = zone_cell[z_to];
            if (tgt >= 0 && dist[tgt] >= 0) {
                bfs_dist[((uint64_t)z_from << 32) | z_to] = dist[tgt];
            }
        }
        
        // Reset dist
        for (int v : visited) dist[v] = -1;
        visited.clear();
    }
    
    inited = true;
}

static inline float get_dist(int a_loc, int tid, int cc, SharedEnvironment* env) {
    int t_loc = env->task_pool[tid].locations[0];
    int az = (a_loc/cc/ZS)*500 + (a_loc%cc/ZS);
    int tz = (t_loc/cc/ZS)*500 + (t_loc%cc/ZS);
    
    if (az == tz) {
        return (float)(std::abs(a_loc/cc - t_loc/cc) + std::abs(a_loc%cc - t_loc%cc));
    }
    
    // Cross-zone: use precomputed BFS between zone centers
    auto it = bfs_dist.find(((uint64_t)az << 32) | tz);
    if (it != bfs_dist.end()) {
        return (float)it->second;
    }
    it = bfs_dist.find(((uint64_t)tz << 32) | az);
    if (it != bfs_dist.end()) {
        return (float)it->second;
    }
    
    // Fallback: Manhattan
    return (float)(std::abs(a_loc/cc - t_loc/cc) + std::abs(a_loc%cc - t_loc%cc));
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;

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

    // V7 zone-interleave ordering (uses 100x100 zones for agent grouping)
    std::unordered_map<int, std::vector<int>> az;
    for (int aa : f_agents) {
        int aaloc = env->curr_states.at(aa).location;
        az[(aaloc/cols/100)*100 + (aaloc%cols/100)].push_back(aa);
    }
    std::vector<int> agents;
    for (bool more = true; more; ) {
        more = false;
        for (auto& kv : az)
            if (!kv.second.empty()) { agents.push_back(kv.second.back()); kv.second.pop_back(); more = true; }
    }

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (!f_agents.count(a)) continue;
        int a_loc = env->curr_states.at(a).location;
        int a_zone100 = (a_loc/cols/100)*100 + (a_loc%cols/100); // 100x100 zone for locality
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) && env->task_pool[cur].locations.size() > 1 && f_tasks.count(cur)) {
            proposed_schedule[a] = cur; f_agents.erase(a); f_tasks.erase(cur); continue;
        }
        int best = -1;
        float best_score = -1e9f;
        for (int tid : f_tasks) {
            // USE REAL BFS DISTANCE (zone-level)
            float travel = get_dist(a_loc, tid, cols, env);
            int tz100 = (env->task_pool[tid].locations[0]/cols/100)*100 + (env->task_pool[tid].locations[0]%cols/100);
            float zbonus = (tz100 == a_zone100) ? 3.0f : 1.0f;
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

} // namespace strategy_zonebfs
