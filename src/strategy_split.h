/// strategy_split.h
/// Architecture 8: 2-zone split (left/right halves) with v6 efficiency
///
/// KEY INSIGHT: With 1800 agents on a game map, MORE regions = MORE fragmentation.
/// Fewer, larger regions reduce the chance of agents crossing zones.
/// 2 zones (left/right halves) with moderate cross penalty.
///
/// This is the OPPOSITE of the topology-BFS approach - simpler is better.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

namespace strategy_split {

static std::unordered_set<int> free_agents_s;
static std::unordered_set<int> free_tasks_s;
static std::unordered_map<int, float> task_value;

void init() { free_agents_s.clear(); free_tasks_s.clear(); task_value.clear(); }

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, mc = cols / 2;

    free_agents_s.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks_s.insert(env->new_tasks.begin(), env->new_tasks.end());

    for (int tid : env->new_tasks) {
        float d = 0;
        for (size_t i = 1; i < env->task_pool[tid].locations.size(); i++)
            d += DefaultPlanner::get_h(env, env->task_pool[tid].locations[i-1], env->task_pool[tid].locations[i]);
        int n = (int)env->task_pool[tid].locations.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_value[tid] = (n == 1) ? base * 1.5f : base;
    }

    std::vector<int> agents(free_agents_s.begin(), free_agents_s.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_s.find(a) == free_agents_s.end()) continue;

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states.at(a).location;
        int agent_side = (a_loc % cols) < mc ? 0 : 1; // left=0, right=1

        for (int tid : free_tasks_s) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            int task_side = (env->task_pool[tid].locations[0] % cols) < mc ? 0 : 1;
            float rp = (task_side == agent_side) ? 1.0f : 2.0f; // mild 2x cross

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

} // namespace strategy_split
