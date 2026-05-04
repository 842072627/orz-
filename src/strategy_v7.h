// strategy_v7.h — 最近优先 + 间隔选择 + 全量扫描
// Phase 1: 50 tasks/agent 快速估距离
// Phase 2: 按距离排序后间隔选择（保持空间多样性，减少 PIBT 拥塞）

#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>

namespace strategy_v7 {

static std::unordered_set<int> free_agents_set;
static std::unordered_set<int> free_tasks_set;
static std::unordered_map<int, float> task_score;

void init() { free_agents_set.clear(); free_tasks_set.clear(); task_score.clear(); }

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols;
    int mr = env->rows / 2, mc = cols / 2;

    free_agents_set.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks_set.insert(env->new_tasks.begin(), env->new_tasks.end());

    for (int tid : env->new_tasks) {
        float d = 0;
        for (size_t i = 1; i < env->task_pool[tid].locations.size(); i++)
            d += DefaultPlanner::get_h(env, env->task_pool[tid].locations[i-1], env->task_pool[tid].locations[i]);
        int n = (int)env->task_pool[tid].locations.size();
        task_score[tid] = (d > 0) ? (float)n / d : 10.0f;
        if (n == 1) task_score[tid] *= 1.5f;
    }

    if (free_agents_set.empty() || free_tasks_set.empty()) return;

    int N = (int)free_tasks_set.size();
    std::vector<int> tasks_vec(free_tasks_set.begin(), free_tasks_set.end());
    int cursor = 0, scan1 = (N < 50) ? N : 50;

    // 快速距离估算
    std::vector<std::pair<float,int>> urgency;
    for (int a : free_agents_set) {
        int a_loc = env->curr_states.at(a).location;
        float min_d = 1e9f;
        for (int k = 0; k < scan1; k++) {
            int tid = tasks_vec[(cursor + k) % N];
            float d = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            if (d < min_d) min_d = d;
        }
        cursor = (cursor + scan1) % N;
        urgency.push_back({min_d, a});
    }
    std::sort(urgency.begin(), urgency.end());

    // 间隔选择：从排序列表中间隔取 agent（保持空间多样性）
    // 取 index 0, 2, 4, 6...（隔一个取一个）
    std::vector<int> selected;
    for (int i = 0; i < (int)urgency.size(); i++) {
        if (i % 2 == 0) selected.push_back(urgency[i].second);
    }
    // 如果还不够，把剩下的也补上
    for (int i = 0; i < (int)urgency.size(); i++) {
        if (i % 2 == 1) selected.push_back(urgency[i].second);
    }

    // 全量扫描分配（直到超时）
    for (int a : selected) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (free_agents_set.find(a) == free_agents_set.end()) continue;

        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_q = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);
        int best = -1;
        float best_s = -1e9f;

        for (int tid : free_tasks_set) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
            int t_q = (env->task_pool[tid].locations[0]/cols < mr ? 0 : 2) + 
                       (env->task_pool[tid].locations[0]%cols < mc ? 0 : 1);
            float rp = (t_q == agent_q) ? 1.0f : 3.0f;
            float s = task_score[tid] / (travel * rp + 1.0f);
            if (s > best_s) { best_s = s; best = tid; }
        }

        if (best >= 0) {
            proposed_schedule[a] = best;
            free_agents_set.erase(a);
            free_tasks_set.erase(best);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_v7
