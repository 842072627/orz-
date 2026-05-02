#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <algorithm>

// Zone-constrained: agents ONLY take tasks in their assigned zone (2 zones: top/bottom half)
// Then within zone, pure distance greedy
static std::unordered_set<int> free_agents_set;
static std::unordered_set<int> free_tasks_top;
static std::unordered_set<int> free_tasks_bot;

void TaskScheduler::initialize(int preprocess_time_limit) {
    free_agents_set.clear(); free_tasks_top.clear(); free_tasks_bot.clear();
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    free_agents_set.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    // Partition new tasks into top/bot
    int cols = env->cols;
    int mid = env->rows / 2;
    for (int tid : env->new_tasks) {
        int tr = env->task_pool[tid].locations[0] / cols;
        if (tr < mid) free_tasks_top.insert(tid);
        else free_tasks_bot.insert(tid);
    }

    std::vector<int> agents(free_agents_set.begin(), free_agents_set.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int best_task = -1;
        float best_dist = 1e9;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols;
        
        // Select task pool based on agent's zone
        std::unordered_set<int>& pool = (ar < mid) ? free_tasks_top : free_tasks_bot;
        // If own zone has no tasks, try the other zone
        std::unordered_set<int>& pool2 = (pool.empty()) ? 
            ((ar < mid) ? free_tasks_bot : free_tasks_top) : pool;

        for (int tid : pool2) {
            int c_loc = a_loc;
            float dist = 0;
            for (int loc : env->task_pool[tid].locations) {
                dist += DefaultPlanner::get_h(env, c_loc, loc); c_loc = loc;
            }
            if (dist < best_dist) { best_dist = dist; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_set.erase(a);
            // Remove from whichever pool
            free_tasks_top.erase(best_task);
            free_tasks_bot.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
