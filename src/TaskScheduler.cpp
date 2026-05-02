#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

// 3x3 = 9 regions, finer granularity
static std::unordered_set<int> free_agents_set;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, int> agent_last_region;

void TaskScheduler::initialize(int preprocess_time_limit) {
    free_agents_set.clear(); free_tasks.clear(); agent_last_region.clear();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    free_agents_set.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int cols = env->cols;
    int m1 = env->rows / 3, m2 = 2 * m1;
    int n1 = cols / 3, n2 = 2 * n1;

    std::vector<int> agents(free_agents_set.begin(), free_agents_set.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int best_task = -1;
        float best_cost = 1e9;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = (ar < m1 ? 0 : (ar < m2 ? 3 : 6)) + (ac < n1 ? 0 : (ac < n2 ? 1 : 2));
        int prev_region = agent_last_region.count(a) ? agent_last_region[a] : -1;

        for (int tid : free_tasks) {
            int c_loc = a_loc;
            float dist = 0;
            for (int loc : env->task_pool[tid].locations) {
                dist += DefaultPlanner::get_h(env, c_loc, loc); c_loc = loc;
            }
            int t_loc = env->task_pool[tid].locations[0];
            int tr = t_loc / cols, tc = t_loc % cols;
            int task_region = (tr < m1 ? 0 : (tr < m2 ? 3 : 6)) + (tc < n1 ? 0 : (tc < n2 ? 1 : 2));

            float region_cost = 0;
            if (task_region != agent_region) region_cost += 2.0f;
            if (prev_region >= 0 && task_region != prev_region) region_cost += 3.0f;

            if (dist + region_cost < best_cost) { best_cost = dist + region_cost; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            int t_loc2 = env->task_pool[best_task].locations[0];
            int tr2 = t_loc2 / cols, tc2 = t_loc2 % cols;
            agent_last_region[a] = (tr2 < m1 ? 0 : (tr2 < m2 ? 3 : 6)) + (tc2 < n1 ? 0 : (tc2 < n2 ? 1 : 2));
            free_agents_set.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
