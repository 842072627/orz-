// strategy_heat.h — V7 BEST: zone-interleave + 3x locality + 2% aging accel 60+
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>

namespace strategy_heat {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() { f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear(); }

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;

    for (int tid : env->new_tasks) {
        if (!tbirth.count(tid)) tbirth[tid] = step;
        float idist = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            idist += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (idist > 0) ? (float)n/idist : 10.0f;
        tval[tid] = (n == 1) ? base * 1.5f : base;
    }

    f_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    f_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (int a = 0; a < env->num_of_agents; a++)
        if (env->curr_task_schedule[a] < 0) f_agents.insert(a);
    if (f_agents.empty() || f_tasks.empty()) {
        for (int a : f_agents) proposed_schedule[a] = -1;
        return;
    }

    std::unordered_map<int, std::vector<int>> az;
    for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
    std::vector<int> agents;
    for (bool more = true; more; ) {
        more = false;
        for (auto& kv : az)
            if (!kv.second.empty()) { agents.push_back(kv.second.back()); kv.second.pop_back(); more = true; }
    }

    for (int a : agents) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (!f_agents.count(a)) continue;
        int a_loc = env->curr_states.at(a).location;
        int a_zone = zid(a_loc, cols);
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) && env->task_pool[cur].locations.size() > 1 && f_tasks.count(cur)) {
            proposed_schedule[a] = cur; f_agents.erase(a); f_tasks.erase(cur); continue;
        }
        int best = -1;
        float best_score = -1e9f;
        for (int tid : f_tasks) {
            int t_loc = env->task_pool[tid].locations[0];
            float travel = DefaultPlanner::get_h(env, a_loc, t_loc);
            int t_zone = zid(t_loc, cols);
            float zbonus = (t_zone == a_zone) ? 3.0f : 1.0f;
            int birth = tbirth.count(tid) ? tbirth[tid] : step;
            int wait = step - birth;
            float age = 1.0f + (float)wait * 0.02f;
            if (wait > 60) age *= (1.0f + (float)(wait - 60) * 0.05f);
            float score = tval[tid] * zbonus * age / (travel + 1.0f);
            if (score > best_score) { best_score = score; best = tid; }
        }
        if (best >= 0) {
            proposed_schedule[a] = best;
            f_agents.erase(a); f_tasks.erase(best);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_heat
