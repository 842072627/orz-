#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <vector>
#include <algorithm>

// Proximity order: agents closest to a task assign first (fastest assignment)
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

    // Build proximity-sorted agent list: agents with a nearby task go first
    std::vector<int> agents(free_agents_set.begin(), free_agents_set.end());
    std::vector<std::pair<float, int>> scored;
    for (int a : agents) {
        float nearest = 1e9;
        int a_loc = env->curr_states.at(a).location;
        for (int tid : free_tasks) {
            float d = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            if (d < nearest) nearest = d;
        }
        scored.push_back({nearest, a});
    }
    std::sort(scored.begin(), scored.end());

    for (auto& [dist, a] : scored) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_set.find(a) == free_agents_set.end()) continue;
        int best_task = -1;
        float best_cost = 1e9;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int a_region = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);

        for (int tid : free_tasks) {
            int c_loc = a_loc;
            float cost = 0;
            for (int loc : env->task_pool[tid].locations) {
                cost += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }
            int tr = env->task_pool[tid].locations[0] / cols;
            int tc = env->task_pool[tid].locations[0] % cols;
            int t_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
            float rp = (t_region == a_region) ? 0.0f : 2.0f;
            if (cost + rp < best_cost) { best_cost = cost + rp; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_set.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
