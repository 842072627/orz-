// strategy_makespan.h — Default scheduler logic (minimum total makespan) + V7 zone-interleaved ordering
// Picks the task with SMALLEST total distance (makespan) for each agent.
// This is what the default scheduler does, but with zone-interleaved agent ordering.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <climits>

namespace strategy_makespan {

static std::unordered_set<int> f_agents, f_tasks;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() { f_agents.clear(); f_tasks.clear(); }

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols;

    f_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    f_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (int a = 0; a < env->num_of_agents; a++)
        if (env->curr_task_schedule[a] < 0) f_agents.insert(a);
    if (f_agents.empty() || f_tasks.empty()) {
        for (int a : f_agents) proposed_schedule[a] = -1;
        return;
    }

    // Zone-interleaved ordering (V7)
    std::unordered_map<int, std::vector<int>> az;
    for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
    std::vector<int> agents;
    for (bool more = true; more; ) {
        more = false;
        for (auto& kv : az)
            if (!kv.second.empty()) { agents.push_back(kv.second.back()); kv.second.pop_back(); more = true; }
    }

    // DEFAULT LOGIC: minimize makespan (total distance to complete all errands)
    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (!f_agents.count(a)) continue;
        int a_loc = env->curr_states.at(a).location;
        int best_task = -1;
        int best_dist = INT_MAX;
        for (int tid : f_tasks) {
            int total = 0;
            int c_loc = a_loc;
            for (int loc : env->task_pool[tid].locations) {
                total += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }
            if (total < best_dist) { best_dist = total; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            f_agents.erase(a); f_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_makespan
