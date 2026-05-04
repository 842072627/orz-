/// strategy_v6plus.h
/// Architecture 5: v6 greedy + congestion heatmap + task aging
/// Uses v6's efficient greedy agent-first approach (no expensive global sort)
/// Adds: zone heatmap penalty + task urgency aging
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <cstring>

namespace strategy_v6plus {

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, float> task_eff;
static std::unordered_map<int, int> task_birth;
static int zone_agent_count[4] = {0,0,0,0};

void init() {
    free_agents.clear(); free_tasks.clear(); task_eff.clear(); task_birth.clear();
    std::memset(zone_agent_count, 0, sizeof(zone_agent_count));
}

int region(int loc, int cols, int mr, int mc) {
    int r = loc / cols, c = loc % cols;
    return (r < mr ? 0 : 2) + (c < mc ? 0 : 1);
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, mr = env->rows / 2, mc = cols / 2;
    int cur_step = env->curr_timestep;

    // NEW TIMESTEP: reset zone heatmap counts
    std::memset(zone_agent_count, 0, sizeof(zone_agent_count));

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    for (int tid : env->new_tasks) {
        task_birth[tid] = cur_step;
        float d = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            d += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_eff[tid] = (n == 1) ? base * 1.5f : base;
    }

    // Track agents by zone for heat map (count agents that will be assigned)
    // Pre-scan: count how many agents are in each zone
    for (int a : free_agents) {
        int a_loc = env->curr_states.at(a).location;
        zone_agent_count[region(a_loc, cols, mr, mc)]++;
    }

    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents.find(a) == free_agents.end()) continue;

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = region(a_loc, cols, mr, mc);

        for (int tid : free_tasks) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            int tr = env->task_pool[tid].locations[0] / cols;
            int tc = env->task_pool[tid].locations[0] % cols;
            int task_region = region(tr*cols+tc, cols, mr, mc);
            float rp = (task_region == agent_region) ? 1.0f : 3.0f;

            // Congestion heat: if cross-region, penalize over-crowded target zones
            float heat = 1.0f;
            if (task_region != agent_region) {
                int zone_agents = zone_agent_count[task_region];
                if (zone_agents > 0) {
                    heat = 1.0f + 0.1f * (float)zone_agents;
                    if (heat > 2.0f) heat = 2.0f;
                }
            }

            // Aging
            int age = cur_step - task_birth[tid];
            float urgency = 1.0f + (float)age * 0.001f;

            float score = task_eff[tid] * urgency / (travel * rp * heat + 1.0f);
            if (score > best_score) { best_score = score; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_v6plus
