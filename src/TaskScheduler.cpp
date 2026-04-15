#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"

// ============================================================
// ARCH-A R3: Greedy scheduler with urgency weighting
// Adds task waiting time as a secondary sort key to reduce
// latency for long-waiting tasks without disrupting the
// primary minimum-makespan heuristic.
// ============================================================

namespace {
std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
}  // anonymous namespace

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
    mt.seed(0);
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
        int min_wait = INT_MAX;   // secondary: task waiting time
        int dist, c_loc, count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
                break;

            dist = 0;
            c_loc = env->curr_states.at(i).location;
            for (int loc : env->task_pool[t_id].locations)
            {
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // Only update if strictly better distance, OR equal distance with longer wait
            if (dist < min_dist ||
                (dist == min_dist && env->task_pool[t_id].t_revealed < min_wait))
            {
                min_task_i = t_id;
                min_dist = dist;
                min_wait = env->task_pool[t_id].t_revealed;
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

    // Remaining agents: no task assigned
    for (int a_id : free_agents)
        proposed_schedule[a_id] = -1;
}
