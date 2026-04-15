#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-A R4: Greedy scheduler with random tie-breaking
// When two tasks have the same distance for an agent, pick
// randomly between them to break symmetry. This reduces
// systematic bias in task assignment patterns and can
// improve throughput on dense maps.
// ============================================================

namespace {
std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
    mt.seed(42);  // fixed seed for reproducibility
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
        int count = 0;
        std::vector<int> best_tasks;  // track ties

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
                best_tasks.clear();
                best_tasks.push_back(t_id);
            }
            else if (dist == min_dist)
            {
                best_tasks.push_back(t_id);
            }
            count++;
        }

        if (!best_tasks.empty())
        {
            // Random tie-break among best tasks
            if (best_tasks.size() > 1)
            {
                std::shuffle(best_tasks.begin(), best_tasks.end(), mt);
                min_task_i = best_tasks.front();
            }
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
