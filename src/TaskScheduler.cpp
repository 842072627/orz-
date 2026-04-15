#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-A R6: Task-ID deterministic tie-breaking
// When two tasks have the same distance for an agent, pick
// the task with smallest ID. This is deterministic (no randomness)
// and avoids both the shuffle bias of R4 and the clustering
// bias of revealed-time ordering in R5.
// ============================================================

namespace {
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
}

void TaskScheduler::plan(int time_limit, std::vector<int> & proposed_schedule)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
            break;

        int i = *it;
        int min_task_i = -1;
        int min_dist = INT_MAX;
        int min_task_id = INT_MAX;  // deterministic tie-break: smallest task ID
        int count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;

            int dist = 0;
            int c_loc = env->curr_states.at(i).location;
            for (int loc : env->task_pool[t_id].locations)
            {
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            if (dist < min_dist)
            {
                min_dist = dist;
                min_task_i = t_id;
                min_task_id = t_id;
            }
            else if (dist == min_dist)
            {
                // Deterministic tie-break: pick smallest task ID
                if (t_id < min_task_id)
                {
                    min_task_id = t_id;
                    min_task_i = t_id;
                }
            }
            count++;
        }

        if (min_task_i != -1)
        {
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        else
        {
            proposed_schedule[i] = -1;
            ++it;
        }
    }

    for (int a_id : free_agents)
        proposed_schedule[a_id] = -1;
}