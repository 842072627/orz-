// strategy_real.h — Incremental BFS distance: compute real path distances from agents to nearby tasks
// On 90% obstacle maps, Manhattan says close but agent can't reach due to walls.
// This fixes that by computing ACTUAL BFS path distances for the critical nearby tasks.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <map>

namespace strategy_real {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

// BFS cache + reusable dist array
static std::unordered_map<uint64_t, int16_t> real_dist;
static std::vector<int16_t> bfs_dist;
static std::vector<int> bfs_visited;  // track which cells were touched (for fast reset)
static const int BFS_LIMIT_CELLS = 500;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() { f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear(); real_dist.clear(); }

// Compute BFS from agent position, limited. REUSES dist array for zero-allocation.
static void cache_real_dist(int agent, int agent_loc, SharedEnvironment* env) {
    int cols = env->cols;
    int N = env->rows * cols;
    
    // Initialize reusable dist array
    if ((int)bfs_dist.size() != N) {
        bfs_dist.assign(N, -1);
        bfs_visited.reserve(BFS_LIMIT_CELLS);
    }
    
    std::queue<int> q;
    q.push(agent_loc);
    bfs_dist[agent_loc] = 0;
    bfs_visited.push_back(agent_loc);
    int explored = 0;
    
    while (!q.empty() && explored < BFS_LIMIT_CELLS) {
        int cur = q.front(); q.pop();
        explored++;
        int16_t nd = bfs_dist[cur] + 1;
        for (int next : DefaultPlanner::global_neighbors[cur]) {
            if (bfs_dist[next] >= 0) continue;
            bfs_dist[next] = nd;
            bfs_visited.push_back(next);
            q.push(next);
        }
    }
    
    // Store distances to tasks found within BFS radius
    for (int tid : f_tasks) {
        int t_loc = env->task_pool[tid].locations[0];
        if (bfs_dist[t_loc] >= 0) {
            uint64_t key = ((uint64_t)agent << 32) | (uint64_t)tid;
            real_dist[key] = bfs_dist[t_loc];
        }
    }
    
    // Reset dist array (only cells we touched)
    for (int cell : bfs_visited) bfs_dist[cell] = -1;
    bfs_visited.clear();
}

static inline float get_real_dist(int agent, int tid, SharedEnvironment* env) {
    uint64_t key = ((uint64_t)agent << 32) | (uint64_t)tid;
    auto it = real_dist.find(key);
    if (it != real_dist.end()) return (float)it->second;
    // Not in cache: the task is FAR from this agent. Use Manhattan × 3 (obstacle correction)
    int cols = env->cols;
    int a_loc = env->curr_states[agent].location;
    int t_loc = env->task_pool[tid].locations[0];
    int man = std::abs(a_loc/cols - t_loc/cols) + std::abs(a_loc%cols - t_loc%cols);
    return (float)man * 3.0f;  // Far tasks: heavy correction
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;

    // Ensure neighbors initialized
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);

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

    // NEW: For each free agent, compute truncated BFS to cache real distances
    // This populates real_dist for nearby tasks (the critical ones)
    for (int a : f_agents) {
        int a_loc = env->curr_states.at(a).location;
        // Only compute if not already cached (lazy: compute once per agent per step)
        cache_real_dist(a, a_loc, env);
    }

    // Urgency-based zone ordering: fewest agents first
    std::map<int, std::vector<int>> az;
    for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
    struct ZI { int cnt; std::vector<int>* v; };
    std::vector<ZI> zones;
    for (auto& kv : az) zones.push_back({(int)kv.second.size(), &kv.second});
    std::sort(zones.begin(), zones.end(), [](auto& a, auto& b) { return a.cnt < b.cnt; });
    std::vector<int> agents;
    for (bool more = true; more; ) {
        more = false;
        for (auto& z : zones)
            if (!z.v->empty()) { agents.push_back(z.v->back()); z.v->pop_back(); more = true; }
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
            float travel = get_real_dist(a, tid, env);  // BFS-cached or Manhattan×3
            int t_zone = zid(env->task_pool[tid].locations[0], cols);
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

} // namespace strategy_real
