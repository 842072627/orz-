// strategy_efficiency.h
// V13: Efficiency + task aging + region affinity
// Score = efficiency * age_boost * single_bonus / (travel * rp + 1)
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, float> task_efficiency;
static std::unordered_map<int, int> task_birth;

void strategy_init() {
    free_agents.clear(); free_tasks.clear(); task_efficiency.clear(); task_birth.clear();
}

void strategy_plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    int cols = env->cols;
    int mr = env->rows / 2, mc = cols / 2;
    int current_step = env->curr_timestep;

    // Precompute efficiency for new tasks
    for (int tid : env->new_tasks) {
        if (task_birth.find(tid) == task_birth.end())
            task_birth[tid] = current_step;

        float internal_dist = 0;
        for (size_t i = 1; i < env->task_pool[tid].locations.size(); i++) {
            internal_dist += DefaultPlanner::get_h(env,
                env->task_pool[tid].locations[i-1],
                env->task_pool[tid].locations[i]);
        }
        int n = (int)env->task_pool[tid].locations.size();
        task_efficiency[tid] = (internal_dist > 0) ? (float)n / internal_dist : 10.0f;
    }

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    std::vector<int> agents(free_agents.begin(), free_agents.end());
    std::sort(agents.begin(), agents.end());

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents.find(a) == free_agents.end()) continue;

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states[a].location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);

        for (int tid : free_tasks) {
            float base_eff = task_efficiency[tid];
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);

            int tr = env->task_pool[tid].locations[0] / cols;
            int tc = env->task_pool[tid].locations[0] % cols;
            int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
            float rp = (task_region == agent_region) ? 1.0f : 3.0f;

            int n = (int)env->task_pool[tid].locations.size();
            float single_bonus = (n == 1) ? 1.5f : 1.0f;

            // Age boost: +0.3% per step waiting
            int birth = task_birth[tid];
            float age_boost = 1.0f + (float)(current_step - birth) * 0.003f;

            float score = base_eff * single_bonus * age_boost / (travel * rp + 1.0f);
            if (score > best_score) { best_score = score; best_task = tid; }
        }
        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}
