#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "heuristics.h"
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <chrono>

namespace {

static std::mt19937 sched_mt;
static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;

struct Assignment {
    int agent;
    int task;
    int cost;
};

} // anonymous namespace

void TaskScheduler::initialize(int preprocess_time_limit)
{
    // Give 30% to scheduler (default gives 50%), leave 70% for planner
    int limit = preprocess_time_limit * 3 / 10 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;
    DefaultPlanner::schedule_initialize(limit, env);
    sched_mt.seed(0);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    // Give 30% to scheduler, 70% to planner
    int limit = time_limit * 3 / 10 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;

    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    // Track free agents and tasks
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Build all agent-task candidates with cost = distance + wait_penalty
    std::vector<Assignment> assignments;

    for (int t_id : free_tasks) {
        if (std::chrono::steady_clock::now() > endtime) break;
        auto it = env->task_pool.find(t_id);
        if (it == env->task_pool.end()) continue;
        auto& task = it->second;
        if (task.agent_assigned != -1) continue;

        for (int i = 0; i < env->num_of_agents; i++) {
            if (proposed_schedule[i] != -1) continue;

            int total_dist = 0;
            int c_loc = env->curr_states.at(i).location;
            for (int loc : task.locations) {
                total_dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;
            }

            // Prefer tasks revealed earlier but still unassigned (avoid starvation)
            int wait_penalty = std::max(0, env->curr_timestep - task.t_revealed) / 5;
            assignments.push_back({i, t_id, total_dist + wait_penalty});
        }
    }

    // Sort by cost ascending (best match first)
    std::sort(assignments.begin(), assignments.end(),
        [](const Assignment& a, const Assignment& b) { return a.cost < b.cost; });

    // Greedy assign: best match first, skip if agent/task already taken
    std::unordered_set<int> assigned_tasks;
    for (auto& a : assignments) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (proposed_schedule[a.agent] != -1) continue;
        if (assigned_tasks.count(a.task)) continue;

        auto it = env->task_pool.find(a.task);
        if (it == env->task_pool.end()) continue;
        if (it->second.agent_assigned != -1) continue;

        proposed_schedule[a.agent] = a.task;
        free_agents.erase(a.agent);
        free_tasks.erase(a.task);
        assigned_tasks.insert(a.task);
        env->task_pool[a.task].agent_assigned = a.agent;
    }

    // Remaining free agents get -1 (no task available)
    for (auto it = free_agents.begin(); it != free_agents.end(); ++it) {
        proposed_schedule[*it] = -1;
    }
    free_agents.clear();
}
