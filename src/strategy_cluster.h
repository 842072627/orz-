/// strategy_cluster.h
/// Architecture 3: Task clustering + agent specialization
///
/// Key insight: Bundle spatially-close tasks into clusters.
/// Each agent specializes in one spatial cluster and services all tasks there.
/// This reduces travel between tasks and improves cache/region affinity.
///
/// Algorithm:
/// 1. Cluster free tasks by spatial proximity (grid-based clustering)
/// 2. Each free agent picks the nearest cluster
/// 3. Agent gets the best task within its chosen cluster
/// 4. Agents that completed their cluster get reassigned to a new one
///
/// This differs from v6 because agents DEVELOP a regional focus,
/// rather than hopping between regions each timestep.
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <cmath>

namespace strategy_cluster {

static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;
static std::unordered_map<int, float> task_value;
static int CLUSTER_SIZE = 10; // grid cells per cluster side

void init() {
    free_agents.clear(); free_tasks.clear(); task_value.clear();
}

int cluster_id(int loc, int cols) {
    int r = loc / cols, c = loc % cols;
    return (r / CLUSTER_SIZE) * 1000 + (c / CLUSTER_SIZE);
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, mr = env->rows / 2, mc = cols / 2;

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // Score tasks
    for (int tid : env->new_tasks) {
        float d = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            d += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (d > 0) ? (float)n / d : 10.0f;
        task_value[tid] = (n == 1) ? base * 1.5f : base;
    }

    // Assign each task to a cluster
    std::unordered_map<int, std::vector<int>> clusters; // cluster_id -> tasks
    for (int tid : free_tasks) {
        int cid = cluster_id(env->task_pool[tid].locations[0], cols);
        clusters[cid].push_back(tid);
    }

    // Sort agents: closest to their nearest cluster first
    std::vector<std::pair<int,int>> agent_cluster; // agent, nearest_cluster_id
    for (int a : free_agents) {
        int a_loc = env->curr_states.at(a).location;
        int best_cid = -1;
        float best_dist = 1e9f;
        for (auto& kv : clusters) {
            if (kv.second.empty()) continue;
            int t0 = kv.second[0];
            float d = DefaultPlanner::get_h(env, a_loc, env->task_pool[t0].locations[0]);
            if (d < best_dist) { best_dist = d; best_cid = kv.first; }
        }
        agent_cluster.push_back({a, best_cid});
    }
    std::sort(agent_cluster.begin(), agent_cluster.end(),
        [&](auto& x, auto& y) {
            if (x.second == y.second && x.second >= 0) return false;
            if (x.second < 0) return false;
            if (y.second < 0) return true;
            int a1 = env->curr_states.at(x.first).location;
            int a2 = env->curr_states.at(y.first).location;
            int t1 = env->task_pool[clusters[x.second][0]].locations[0];
            int t2 = env->task_pool[clusters[y.second][0]].locations[0];
            return DefaultPlanner::get_h(env, a1, t1) < DefaultPlanner::get_h(env, a2, t2);
        });

    // Assign: each agent picks best task from its nearest cluster
    for (auto& ac : agent_cluster) {
        if (std::chrono::steady_clock::now() > endtime) break;
        int a = ac.first;
        if (free_agents.find(a) == free_agents.end()) continue;

        int best_task = -1;
        float best_score = -1e9f;
        int a_loc = env->curr_states.at(a).location;
        int ar = a_loc / cols, ac2 = a_loc % cols;
        int agent_region = (ar < mr ? 0 : 2) + (ac2 < mc ? 0 : 1);

        // Try tasks from nearest cluster first
        int cid = ac.second;
        std::vector<int>* task_set = (cid >= 0 && clusters.count(cid)) ? &clusters[cid] : nullptr;

        // If cluster exists, search it
        if (task_set) {
            for (int tid : *task_set) {
                if (free_tasks.find(tid) == free_tasks.end()) continue;
                float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
                int tr = env->task_pool[tid].locations[0] / cols;
                int tc = env->task_pool[tid].locations[0] % cols;
                int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
                float rp = (task_region == agent_region) ? 1.0f : 3.0f;
                float score = task_value[tid] / (travel * rp + 1.0f);
                if (score > best_score) { best_score = score; best_task = tid; }
            }
        }

        // If no task found in cluster, search ALL tasks as fallback
        if (best_task < 0) {
            for (int tid : free_tasks) {
                float travel = DefaultPlanner::get_h(env, a_loc, env->task_pool[tid].locations[0]);
                int tr = env->task_pool[tid].locations[0] / cols;
                int tc = env->task_pool[tid].locations[0] % cols;
                int task_region = (tr < mr ? 0 : 2) + (tc < mc ? 0 : 1);
                float rp = (task_region == agent_region) ? 1.0f : 3.0f;
                float score = task_value[tid] / (travel * rp + 1.0f);
                if (score > best_score) { best_score = score; best_task = tid; }
            }
        }

        if (best_task >= 0) {
            proposed_schedule[a] = best_task;
            free_agents.erase(a);
            free_tasks.erase(best_task);
        } else proposed_schedule[a] = -1;
    }
}

} // namespace strategy_cluster
