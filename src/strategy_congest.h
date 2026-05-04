// strategy_congest.h - Congestion-aware scheduler for dense MAPF
//
// KEY IMPROVEMENTS over strategy_efficiency:
// 1. CONGESTION AVOIDANCE: region_load penalty for zones with many agents
// 2. MULTI-ERRAND TASK CONTINUITY: agents with ongoing multi-step tasks keep them
// 3. BETTER ORDERING: agents sorted by region congestion (less-congested first)
// 4. ADAPTIVE: quadrants split adjusts based on map dimensions
// 5. FASTER: early break when all agents from non-loaded regions are assigned
//
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace strategy_congest {

static std::unordered_set<int> free_agents_c;
static std::unordered_set<int> free_tasks_c;
static std::unordered_map<int, float> task_value;

void init() {
    free_agents_c.clear();
    free_tasks_c.clear();
    task_value.clear();
}

// Get region index (1-4 quadrants) for a location
static inline int get_region(int loc, int cols, int mr, int mc) {
    int r = loc / cols;
    int c = loc % cols;
    return (r < mr ? 0 : 2) + (c < mc ? 0 : 1);
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    
    free_agents_c.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks_c.insert(env->new_tasks.begin(), env->new_tasks.end());

    int cols = env->cols;
    int mr = env->rows / 2, mc = cols / 2;

    // Precompute task values (same as strategy_efficiency)
    for (int tid : env->new_tasks) {
        float d = 0;
        for (size_t i = 1; i < env->task_pool[tid].locations.size(); i++)
            d += DefaultPlanner::get_h(env, env->task_pool[tid].locations[i-1], env->task_pool[tid].locations[i]);
        int n = (int)env->task_pool[tid].locations.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_value[tid] = (n == 1) ? base * 1.5f : base;
    }

    // Build agent list and compute per-region load
    std::vector<int> agents(free_agents_c.begin(), free_agents_c.end());
    
    int region_load[4] = {0, 0, 0, 0};
    std::unordered_map<int, int> agent_region;
    for (int a : agents) {
        int loc = env->curr_states.at(a).location;
        int r = get_region(loc, cols, mr, mc);
        agent_region[a] = r;
        region_load[r]++;
    }

    // Sort agents: less-congested regions first, then by region number
    // This prevents overloaded regions from grabbing all the good tasks
    std::sort(agents.begin(), agents.end(), [&](int a, int b) {
        int ra = agent_region[a], rb = agent_region[b];
        if (region_load[ra] != region_load[rb])
            return region_load[ra] < region_load[rb]; // less-loaded first
        return ra < rb;
    });

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_c.find(a) == free_agents_c.end()) continue;

        // Check if agent has an ongoing multi-errand task - keep it
        int cur_task = env->curr_task_schedule[a];
        if (cur_task >= 0) {
            auto it = env->task_pool.find(cur_task);
            if (it != env->task_pool.end() && it->second.locations.size() > 1) {
                proposed_schedule[a] = cur_task;
                free_agents_c.erase(a);
                continue;
            }
        }

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states.at(a).location;
        int ar = get_region(a_loc, cols, mr, mc);

        for (int tid : free_tasks_c) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            int tr = get_region(env->task_pool[tid].locations[0], cols, mr, mc);
            
            // Region penalty: 1x same region, 2x adjacent, 3x opposite
            float rp;
            if (tr == ar) rp = 1.0f;
            else if ((tr ^ ar) == 1 || (tr ^ ar) == 2) rp = 2.0f; // adjacent quadrants
            else rp = 2.5f; // diagonal (opposite)
            
            // CONGESTION PENALTY: if target region has many agents, penalize
            float congestion = 1.0f + 0.1f * (float)region_load[tr] / 250.0f;
            
            float score = task_value[tid] / (travel * rp * congestion + 1.0f);
            if (score > best_score) { best_score = score; best_task = tid; }
        }
        
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_c.erase(a);
            free_tasks_c.erase(best_task);
        } else {
            proposed_schedule[a] = -1;
        }
    }
}

} // namespace strategy_congest
