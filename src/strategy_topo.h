/// strategy_topo.h
/// Architecture 7: Topology-aware scheduler with precomputed regions
///
/// KEY INNOVATION: Instead of 4 hard grid quadrants, compute ACTUAL map
/// connectivity regions in initialize(). This respects walls, corridors, rooms.
///
/// Algorithm:
/// 1. INITIALIZE: BFS from agent start positions to find natural regions
///    - Sample random walkable cells as region seeds
///    - Flood-fill to assign cells to regions
///    - Record each agent's home region
/// 2. PLAN: Use region affinity from topology, not arbitrary grid
///    - Score = task_value / (travel * region_penalty + 1)
///    - region_penalty based on whether task is in agent's home region
///    - Same efficient greedy as v6, but with BETTER region definitions
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <cstring>

namespace strategy_topo {

static std::unordered_set<int> free_agents_s;
static std::unordered_set<int> free_tasks_s;
static std::unordered_map<int, float> task_value;

// Topology data (computed once in init)
static std::vector<int> cell_region;   // map cell -> region id (-1 for obstacles)
static std::vector<int> agent_home_region; // agent -> home region
static int num_regions = 0;
static bool topo_ready = false;

void init() {
    free_agents_s.clear(); free_tasks_s.clear(); task_value.clear();
    cell_region.clear(); agent_home_region.clear();
    num_regions = 0; topo_ready = false;
}

void topo_initialize(SharedEnvironment* env) {
    int rows = env->rows, cols = env->cols, N = rows * cols;
    cell_region.assign(N, -1);
    
    // Collect all walkable cells
    std::vector<int> walkable;
    for (int i = 0; i < N; i++)
        if (env->map[i] == 0) walkable.push_back(i);
    
    if (walkable.empty()) return;
    
    // Use AGENT START POSITIONS as region seeds (guarantees each agent has a region)
    // This is the KEY difference from grid-based partitioning: regions grow from
    // where agents actually are, following map topology
    std::queue<int> bfs_q;
    int region_id = 0;
    
    for (int a = 0; a < env->num_of_agents && a < (int)env->start_states.size(); a++) {
        int seed = env->start_states[a].location;
        if (seed < 0 || seed >= N) continue;
        if (cell_region[seed] >= 0) continue; // already assigned
        
        // BFS flood-fill from this seed
        region_id++;
        bfs_q.push(seed);
        cell_region[seed] = region_id;
        
        const int MAX_CELLS_PER_REGION = (int)(walkable.size() / 4) + 1;
        int cells_in_region = 0;
        
        while (!bfs_q.empty() && cells_in_region < MAX_CELLS_PER_REGION) {
            int cur = bfs_q.front(); bfs_q.pop();
            cells_in_region++;
            int r = cur / cols, c = cur % cols;
            
            // 4-directional neighbors
            int nbrs[4] = {cur-1, cur+1, cur-cols, cur+cols};
            for (int nb : nbrs) {
                if (nb < 0 || nb >= N) continue;
                if (env->map[nb] != 0) continue; // obstacle
                if (cell_region[nb] >= 0) continue; // already assigned
                cell_region[nb] = region_id;
                bfs_q.push(nb);
            }
        }
        // Clear queue for next seed
        while (!bfs_q.empty()) bfs_q.pop();
    }
    
    // Assign remaining unassigned walkable cells to nearest region
    for (int loc : walkable) {
        if (cell_region[loc] >= 0) continue;
        // Find nearest assigned cell and copy its region
        int best_dist = 1e9, best_region = 1;
        for (int other : walkable) {
            if (cell_region[other] < 0) continue;
            int d = DefaultPlanner::get_h(env, loc, other);
            if (d < best_dist) { best_dist = d; best_region = cell_region[other]; }
        }
        cell_region[loc] = best_region;
    }
    
    num_regions = region_id;
    
    // Assign each agent to its home region (from start position)
    agent_home_region.assign(env->num_of_agents, 1);
    for (int a = 0; a < env->num_of_agents && a < (int)env->start_states.size(); a++) {
        int loc = env->start_states[a].location;
        if (loc >= 0 && loc < N && cell_region[loc] >= 0)
            agent_home_region[a] = cell_region[loc];
    }
    
    topo_ready = true;
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols;
    
    // Lazy topology init (first call only)
    if (!topo_ready) {
        topo_initialize(env);
    }
    
    free_agents_s.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks_s.insert(env->new_tasks.begin(), env->new_tasks.end());

    for (int tid : env->new_tasks) {
        float d = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            d += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_value[tid] = (n == 1) ? base * 1.5f : base;
    }

    // Sort agents: agents in their HOME region get priority (they should serve nearby tasks)
    std::vector<int> agents(free_agents_s.begin(), free_agents_s.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_s.find(a) == free_agents_s.end()) continue;

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states.at(a).location;
        int a_home = (a < (int)agent_home_region.size()) ? agent_home_region[a] : 0;

        for (int tid : free_tasks_s) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            int t_loc = env->task_pool[tid].locations[0];
            
            // Get task's region from topology
            int task_reg = (t_loc >= 0 && t_loc < (int)cell_region.size()) ? cell_region[t_loc] : 0;
            
            // Region penalty: based on TOPOLOGY regions, not grid!
            float rp = (task_reg == a_home) ? 1.0f : 3.0f;

            float score = task_value[tid] / (travel * rp + 1.0f);
            if (score > best_score) { best_score = score; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_s.erase(a);
            free_tasks_s.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_topo
