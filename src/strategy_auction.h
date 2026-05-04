/// strategy_auction.h
/// Architecture 2: Auction-based task assignment
/// 
/// Key insight: Instead of greedy agent-first assignment (each agent picks best task),
/// use a TASK-FIRST auction: for each task, find the best agent, then resolve conflicts.
/// This gives a MORE OPTIMAL global assignment than greedy agent ordering.
///
/// Algorithm:
/// 1. Compute agent-task compatibility scores (same as v6 formula)
/// 2. Sort ALL pairs by score DESCENDING (global view, not per-agent)
/// 3. Assign best pairs first, conflict-free
/// 4. Remaining unassigned agents get fallback greedy
///
/// This eliminates the "lucky first agent" effect of ID-sorted greedy.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>

namespace strategy_auction {

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, float> task_value;

void init() {
    free_agents.clear(); free_tasks.clear(); task_value.clear();
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, mr = env->rows / 2, mc = cols / 2;

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Phase 1: Score tasks
    for (int tid : env->new_tasks) {
        float d = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            d += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_value[tid] = (n == 1) ? base * 1.5f : base;
    }

    // Phase 2: Build and sort ALL agent-task pairs
    struct Bid { int agent; int task; float score; };
    std::vector<Bid> bids;
    bids.reserve(free_agents.size() * free_tasks.size());

    for (int a : free_agents) {
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac = a_loc % cols;
        int agent_region = (ar < mr ? 0 : 2) + (ac < mc ? 0 : 1);

        for (int t : free_tasks) {
            float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[t].locations[0]);
            int tr = env->task_pool[t].locations[0] / cols;
            int tc = env->task_pool[t].locations[0] % cols;
            int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
            float rp = (task_region == agent_region) ? 1.0f : 3.0f;
            bids.push_back({a, t, task_value[t] / (travel * rp + 1.0f)});
        }
    }

    std::sort(bids.begin(), bids.end(), [](const Bid& x, const Bid& y) {
        return x.score > y.score;
    });

    // Phase 3: Assign best bids, conflict-free
    std::unordered_set<int> assigned_agents, assigned_tasks;
    for (auto& bid : bids) {
        if (std::chrono::steady_clock::now() > endtime) break;
        if (assigned_agents.count(bid.agent)) continue;
        if (assigned_tasks.count(bid.task)) continue;
        if (free_agents.find(bid.agent) == free_agents.end()) continue;
        if (free_tasks.find(bid.task) == free_tasks.end()) continue;

        proposed_schedule[bid.agent] = bid.task;
        assigned_agents.insert(bid.agent);
        assigned_tasks.insert(bid.task);
        free_agents.erase(bid.agent);
        free_tasks.erase(bid.task);
    }

    // Phase 4: Remaining agents get -1
    for (int a : free_agents) {
        if (proposed_schedule[a] == -1) proposed_schedule[a] = -1;
    }
}

} // namespace strategy_auction
