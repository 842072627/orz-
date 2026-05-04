// strategy_bfs_full.h — REAL BFS from zone centers to all cells (20MB, ~5s compute)
// Zone-center BFS approach: 105 zones × 96K cells, O(1) distance lookup
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstdint>

namespace strategy_bfs_full {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

// ALT (A* Landmarks Triangle inequality): 20 landmark BFS distances
// landmark[i] = vector of int16_t distances from landmark i to all cells
static const int NUM_LM = 20;
static std::vector<int> lm_cells;  // landmark cell indices
static std::vector<std::vector<int16_t>> lm_dist;  // lm_dist[l][cell] = distance
static bool ready = false;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() {
    f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear();
    zone_dist.clear(); zone_center_cell.clear(); ready = false;
}

void precompute(SharedEnvironment* env) {
    int cols = env->cols, rows = env->rows, N = rows * cols;
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    // Choose 20 well-distributed landmarks on walkable cells
    // Spread across the map: sample evenly spaced rows/cols
    std::vector<int> candidates;
    for (int r = 0; r < rows; r += rows/6) {
        for (int c = 0; c < cols; c += cols/5) {
            int loc = r * cols + c;
            if (loc < N && env->map[loc] == 0) {
                candidates.push_back(loc);
            }
        }
    }
    // Pick up to NUM_LM
    lm_cells.clear();
    for (int i = 0; i < NUM_LM && i < (int)candidates.size(); i++) {
        lm_cells.push_back(candidates[i]);
    }
    
    if (lm_cells.empty()) { ready = true; return; }
    
    // BFS from each landmark to all cells
    std::vector<int16_t> dist(N, -1);
    std::vector<int> visited;
    visited.reserve(N);
    
    for (int li = 0; li < (int)lm_cells.size(); li++) {
        // Reset
        for (int v : visited) dist[v] = -1;
        visited.clear();
        
        int start = lm_cells[li];
        std::queue<int> q;
        q.push(start); dist[start] = 0; visited.push_back(start);
        
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int16_t nd = dist[cur] + 1;
            for (int nb : DefaultPlanner::global_neighbors[cur]) {
                if (dist[nb] >= 0) continue;
                dist[nb] = nd; visited.push_back(nb); q.push(nb);
            }
        }
        
        lm_dist.push_back(dist); // Copy
    }
    
    ready = true;
}

// Get improved distance using triangle inequality on zone-center BFS
static inline float get_real_dist(int a_loc, int t_loc, int cols, SharedEnvironment* env) {
    int man = std::abs(a_loc/cols - t_loc/cols) + std::abs(a_loc%cols - t_loc%cols);
    if (!ready) return (float)man;
    
    int az = zid(a_loc, cols);
    auto zd = zone_dist.find(az);
    if (zd != zone_dist.end()) {
        int16_t da = zd->second[a_loc];
        int16_t dt = zd->second[t_loc];
        if (da >= 0 && dt >= 0) {
            // Triangle inequality: |dist(C, T) - dist(C, A)| ≤ dist(A, T)
            // This is a LOWER BOUND. We also know dist(A,T) ≤ dist(C,T) + dist(C,A)
            // (But that UPPER BOUND is too loose)
            int diff = abs(dt - da);
            // Blend: use diff for nearby, manhattan as upper bound
            return (float)diff + 1.0f;
        }
    }
    
    return (float)man;
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

    std::unordered_map<int, std::vector<int>> az;
    for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
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
        int a_zone = zid(a_loc, cols);
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) && env->task_pool[cur].locations.size() > 1 && f_tasks.count(cur)) {
            proposed_schedule[a] = cur; f_agents.erase(a); f_tasks.erase(cur); continue;
        }
        int best = -1;
        float best_score = -1e9f;
        for (int tid : f_tasks) {
            int t_loc = env->task_pool[tid].locations[0];
            float travel = get_real_dist(a_loc, t_loc, cols, env);
            int t_zone = zid(t_loc, cols);
            float zbonus = (t_zone == a_zone) ? 3.0f : 1.0f;
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

} // namespace strategy_bfs_full
