/// strategy_hybrid.h
/// Architecture 4: Hybrid scheduler - combines ALL proven techniques
///
/// Components:
/// 1. Global pair sorting (eliminates agent-order bias)
/// 2. 4-zone congestion heatmap (penalize overcrowded zones)
/// 3. Task aging (older tasks get urgency boost)
/// 4. Single-errand priority bonus
/// 5. Workload spreading (avoid sending more agents to same zone)
///
/// Formula: score = task_eff * urgency * single_bonus / (travel * rp * heat_penalty + 1)
///   task_eff = n_errands / internal_dist  (or 10 for single-location)
///   urgency = 1 + age * 0.002  (aging bonus)
///   single_bonus = 1.5x for single-errand
///   rp = 1 (same zone), 3 (cross zone)
///   heat_penalty = 1 + 0.1 * (agents_in_zone - 3)  for non-agent-own zones
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cstring>

namespace strategy_hybrid {

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, float> task_eff;
static std::unordered_map<int, int> task_birth; // timestep when each task appeared
static int zone_agent_count[4] = {0,0,0,0}; // agents currently assigned to each zone
static int cols_cache = 0, mr_cache = 0, mc_cache = 0;

int get_region(int loc, int cols) {
    int r = loc / cols, c = loc % cols;
    return (r < mr_cache ? 0 : 2) + (c < mc_cache ? 0 : 1);
}

void init() {
    free_agents.clear(); free_tasks.clear(); task_eff.clear(); task_birth.clear();
    std::memset(zone_agent_count, 0, sizeof(zone_agent_count));
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    cols_cache = env->cols;
    mr_cache = env->rows / 2;
    mc_cache = cols_cache / 2;
    int cur_step = env->curr_timestep;

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Score new tasks + record birth times
    for (int tid : env->new_tasks) {
        task_birth[tid] = cur_step;
        float d = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            d += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_eff[tid] = base; // without single bonus (applied per-task)
        (void)n; // used in scoring formula
    }

    // Build ALL agent-task pairs with rich scoring
    struct Bid { int agent; int task; float score; };
    std::vector<Bid> bids;
    bids.reserve(free_agents.size() * free_tasks.size());

    for (int a : free_agents) {
        int a_loc = env->curr_states.at(a).location;
        int agent_region = get_region(a_loc, cols_cache);

        for (int t : free_tasks) {
            auto& task = env->task_pool[t];
            float travel = DefaultPlanner::get_h(env, a_loc, task.locations[0]);
            int task_region = get_region(task.locations[0], cols_cache);

            // Region penalty
            float rp = (task_region == agent_region) ? 1.0f : 3.0f;

            // Congestion heat penalty: penalize sending agents to already-crowded zones
            // Zone heat = number of agents already assigned to that task's region
            // Only apply heat to cross-region assignments (keep agents local)
            float heat = 1.0f;
            if (task_region != agent_region) {
                // Progressive heat penalty: more agents in zone = harder to assign cross-zone
                heat = 1.0f + 0.15f * (float)zone_agent_count[task_region];
                if (heat > 2.5f) heat = 2.5f;
            }

            // Task efficiency (errands/density)
            float eff = task_eff[t];
            int n = (int)task.locations.size();
            float sb = (n == 1) ? 1.5f : 1.0f;

            // Aging: +0.2% per timestep since birth
            int age_steps = cur_step - task_birth[t];
            float urgency = 1.0f + (float)age_steps * 0.002f;

            // Final score
            float score = eff * sb * urgency / (travel * rp * heat + 1.0f);
            bids.push_back({a, t, score});
        }
    }

    // Sort by score descending (global optimal assignment)
    std::sort(bids.begin(), bids.end(), [](const Bid& x, const Bid& y) {
        return x.score > y.score;
    });

    // Assign best pairs, conflict-free
    for (auto& bid : bids) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (proposed_schedule[bid.agent] != -1) continue;
        if (free_tasks.find(bid.task) == free_tasks.end()) continue;
        if (free_agents.find(bid.agent) == free_agents.end()) continue;

        proposed_schedule[bid.agent] = bid.task;
        free_agents.erase(bid.agent);
        free_tasks.erase(bid.task);

        // Update zone heatmap
        int tr = get_region(env->task_pool[bid.task].locations[0], cols_cache);
        zone_agent_count[tr]++;
    }

    // Remaining get -1
    for (int a : free_agents)
        proposed_schedule[a] = -1;
}

} // namespace strategy_hybrid
