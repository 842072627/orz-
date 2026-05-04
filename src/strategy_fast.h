// strategy_fast.h - Fast makespan scheduler with bounded scan
//
// Uses the DEFAULT scheduler's min-makespan formula (simplest/fastest eval)
// but pre-sorts tasks by base distance to bound per-agent scan cost.
// 
// With SCAN_LIMIT=500: 4000*500=2M evals, fits in 500ms budget.
// This gives ALL agents a chance to be assigned every timestep.
//
// CORE FORMULA: min_makespan (same as default scheduler)
// - Makespan = sum of heuristic distances between task errand locations
// - Select task with minimum makespan for each agent
// - Cross-region assignments allowed (no penalty)
// - Single-errand tasks prioritized inherently (shorter makespan)

#pragma once
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <vector>
#include <algorithm>

namespace strategy_fast {

static std::unordered_set<int> free_agents_f;
static std::unordered_set<int> free_tasks_f;

struct TaskInfo {
    int id;
    float base_makespan;  // sum of distances between errand locations
    int first_loc;
};
static std::vector<TaskInfo> task_infos;
static bool infos_dirty = false;

void init() {
    free_agents_f.clear();
    free_tasks_f.clear();
    task_infos.clear();
    infos_dirty = true;
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    free_agents_f.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    if (!env->new_tasks.empty()) {
        infos_dirty = true;
        for (int tid : env->new_tasks) {
            free_tasks_f.insert(tid);
            const auto& locs = env->task_pool[tid].locations;
            float makespan = 0.0f;
            for (size_t i = 1; i < locs.size(); i++)
                makespan += (float)DefaultPlanner::get_h(env, locs[i-1], locs[i]);
            task_infos.push_back({tid, makespan, locs[0]});
        }
    }

    if (infos_dirty && !task_infos.empty()) {
        // Sort by first_loc as a crude spatial hash for better task distribution
        // This helps PIBT by spreading agents across locations
        std::sort(task_infos.begin(), task_infos.end(), [](const TaskInfo& a, const TaskInfo& b) {
            return a.first_loc < b.first_loc;
        });
        infos_dirty = false;
    }

    if (free_agents_f.empty() || free_tasks_f.empty()) return;

    std::vector<int> agents(free_agents_f.begin(), free_agents_f.end());
    std::sort(agents.begin(), agents.end());

    const int SCAN_LIMIT = 500;  // Big enough for good assignments, fast enough for 4000 agents

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_f.find(a) == free_agents_f.end()) continue;

        int best_task = -1;
        float best_makespan = 1e30f;
        int a_loc = env->curr_states.at(a).location;

        // Find task with minimum TOTAL makespan (travel to first + internal) 
        // from the sorted list, scanning up to SCAN_LIMIT tasks
        int scanned = 0;
        for (const auto& ti : task_infos) {
            if (free_tasks_f.find(ti.id) == free_tasks_f.end()) continue;
            if (++scanned > SCAN_LIMIT) break;

            float travel = (float)DefaultPlanner::get_h(env, a_loc, ti.first_loc);
            if (travel < 1.0f) travel = 1.0f;
            float total = travel + ti.base_makespan;

            if (total < best_makespan) {
                best_makespan = total;
                best_task = ti.id;
            }
        }

        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents_f.erase(a);
            free_tasks_f.erase(best_task);
        } else {
            proposed_schedule[a] = -1;
        }
    }
}

} // namespace strategy_fast
