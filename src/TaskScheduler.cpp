#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

// Hybrid: region_focus + distance-first agents + task assignment without erasing from master pool til end
static std::unordered_set<int> free_agents_set;
static std::unordered_set<int> free_tasks;

void TaskScheduler::initialize(int preprocess_time_limit) {
    free_agents_set.clear(); free_tasks.clear();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    free_agents_set.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int cols = env->cols;
    int mr = env->rows / 2, mc = cols / 2;

    // Sort agents by proximity to their nearest task (closest first)
    std::vector<std::pair<float,int>> agent_order;
    for (int a : free_agents_set) {
        float min_d = 1e9;
        for (int tid : free_tasks) {
            float d = DefaultPlanner::get_h(env, env->curr_states.at(a).location, env->task_pool[tid].locations[0]);
            if (d < min_d) min_d = d;
        }
        agent_order.push_back({min_d, a});
    }
    std::sort(agent_order.begin(), agent_order.end());

    for (auto& [_, a] : agent_order) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int best_task = -1;
        float best_cost = 1e9;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);

        for (int tid : free_tasks) {
            int c_loc = a_loc;
            float dist = 0;
            for (int loc : env->task_pool[tid].locations) {
                dist += DefaultPlanner::get_h(env, c_loc, loc); c_loc = loc;
            }
            int tr = env->task_pool[tid].locations[0] / cols;
            int tc = env->task_pool[tid].locations[0] % cols;
            int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
            float rc = (task_region == agent_region) ? 0 : 2.0f;
            if (dist + rc < best_cost) { best_cost = dist + rc; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_set.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
