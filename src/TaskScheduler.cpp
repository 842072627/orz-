#include "TaskScheduler.h"

#include "scheduler.h"
#include "const.h"
#include <cmath>
#include <algorithm>
#include <random>

// ============================================================
// ARCH-A R2: Auction-based Task Scheduler with Congestion-Aware Bids
// Key changes vs greedy baseline:
//   - K=16: consider top-K agents per task (not all agents)
//   - Bid = distance + congestion_penalty(agent, task)
//   - Congestion: penalize agents near high-density task areas
// ============================================================

namespace {

const int K_BID = 16;              // top-K agents to evaluate per task
const int CONG_FACTOR = 30;        // congestion penalty multiplier
const int MAX_TASKS_PER_BATCH = 500; // max tasks to auction per timestep

// Persistent state across timesteps
std::mt19937 rng;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::vector<int> congestion_map;   // location -> congestion count

inline int agent_distance(SharedEnvironment* env, int agent_id, int task_id) {
    int dist = 0;
    int c_loc = env->curr_states.at(agent_id).location;
    for (int loc : env->task_pool[task_id].locations) {
        dist += DefaultPlanner::get_h(env, c_loc, loc);
        c_loc = loc;
    }
    return dist;
}

// Compute congestion factor: how many other agents are targeting nearby task locations
inline int congestion_penalty(SharedEnvironment* env, int agent_id, int task_id) {
    int agent_loc = env->curr_states.at(agent_id).location;
    int penalty = 0;
    // Count agents within congestion radius of task's first location
    const int& first_loc = env->task_pool[task_id].locations.at(0);
    for (const auto& state : env->curr_states) {
        int d = DefaultPlanner::get_h(env, state.location, first_loc);
        if (d < 10) {  // within 10 steps
            penalty += (10 - d);  // closer = higher penalty
        }
    }
    return penalty * CONG_FACTOR / 10;
}

}  // anonymous namespace

void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::schedule_initialize(limit, env);
    rng.seed(0);
    congestion_map.assign(env->cols * env->rows, 0);
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    // Collect free agents and tasks
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int n_agents = free_agents.size();
    int n_tasks = free_tasks.size();
    if (n_agents == 0 || n_tasks == 0) {
        // No assignment needed, fill remaining with -1
        for (int i : free_agents) {
            proposed_schedule[i] = -1;
        }
        return;
    }

    // Build vectors for faster iteration
    std::vector<int> agent_vec(free_agents.begin(), free_agents.end());
    std::vector<int> task_vec(free_tasks.begin(), free_tasks.end());
    if (task_vec.size() > MAX_TASKS_PER_BATCH) {
        std::shuffle(task_vec.begin(), task_vec.end(), rng);
        task_vec.resize(MAX_TASKS_PER_BATCH);
    }

    // Auction: for each task, find best agent (lowest bid = dist + congestion)
    // Then greedily assign in order of increasing bid
    struct BidRecord {
        int task_id;
        int agent_id;
        int bid;
    };
    std::vector<BidRecord> bids;

    for (int t_id : task_vec) {
        // Find top-K nearest agents for this task
        std::vector<std::pair<int, int>> agent_dists;  // (agent_id, dist)
        agent_dists.reserve(agent_vec.size());
        for (int a_id : agent_vec) {
            int dist = agent_distance(env, a_id, t_id);
            agent_dists.emplace_back(a_id, dist);
        }
        // Partial sort: keep only top-K by distance
        if ((int)agent_dists.size() > K_BID) {
            std::nth_element(agent_dists.begin(), agent_dists.begin() + K_BID, agent_dists.end(),
                             [](const auto& a, const auto& b) { return a.second < b.second; });
            agent_dists.resize(K_BID);
        }
        // Compute full bid (dist + congestion) for top-K
        for (const auto& ad : agent_dists) {
            int a_id = ad.first;
            int dist = ad.second;
            int cong = congestion_penalty(env, a_id, t_id);
            int bid = dist + cong;
            bids.push_back({t_id, a_id, bid});
        }
    }

    // Sort bids by increasing bid value
    std::sort(bids.begin(), bids.end(), [](const BidRecord& a, const BidRecord& b) {
        return a.bid < b.bid;
    });

    // Greedy assignment: each agent/task can only be assigned once
    std::unordered_set<int> assigned_agents;
    std::unordered_set<int> assigned_tasks;

    for (const auto& b : bids) {
        if (assigned_agents.count(b.agent_id) || assigned_tasks.count(b.task_id)) continue;
        if (std::chrono::steady_clock::now() > endtime) break;

        proposed_schedule[b.agent_id] = b.task_id;
        assigned_agents.insert(b.agent_id);
        assigned_tasks.insert(b.task_id);

        // Remove from free sets
        free_agents.erase(b.agent_id);
        free_tasks.erase(b.task_id);
    }

    // Remaining agents get -1
    for (int a_id : free_agents) {
        proposed_schedule[a_id] = -1;
    }

    // Update congestion map based on new assignments
    for (int a_id : assigned_agents) {
        int t_id = proposed_schedule[a_id];
        if (t_id >= 0 && !env->task_pool[t_id].locations.empty()) {
            int loc = env->task_pool[t_id].locations[0];
            if (loc >= 0 && loc < (int)congestion_map.size()) {
                congestion_map[loc]++;
            }
        }
    }

    // Decay congestion map
    for (auto& c : congestion_map) {
        if (c > 0) c--;
    }
}
