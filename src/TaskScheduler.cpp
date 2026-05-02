#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

// Efficiency focus: prefer tasks that can be completed QUICKLY (short makespan) 
// to maximize throughput per unit time
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

    // Pre-compute task efficiency: small_tasks / short_distance = good
    std::unordered_map<int, float> task_efficiency;
    for (int tid : free_tasks) {
        int n_errands = (int)env->task_pool[tid].locations.size();
        float total_dist = 0;
        for (size_t i = 1; i < env->task_pool[tid].locations.size(); i++) {
            total_dist += DefaultPlanner::get_h(env, env->task_pool[tid].locations[i-1], env->task_pool[tid].locations[i]);
        }
        // Efficiency = number of errands / internal distance (tasks that are compact are efficient)
        task_efficiency[tid] = total_dist > 0 ? (float)n_errands / total_dist : 10.0f;
    }

    std::vector<int> agents(free_agents_set.begin(), free_agents_set.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int best_task = -1;
        float best_score = -1e9;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);

        for (int tid : free_tasks) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            // Score = efficiency / (travel + region_penalty). High score = good match
            float rp = 1.0f;
            int tr = env->task_pool[tid].locations[0] / cols;
            int tc = env->task_pool[tid].locations[0] % cols;
            int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
            if (task_region != agent_region) rp = 3.0f;
            
            float score = task_efficiency[tid] / (travel * rp + 1.0f);
            if (score > best_score) { best_score = score; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_set.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
