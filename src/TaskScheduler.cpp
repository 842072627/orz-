#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include "heuristics.h"
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <chrono>
#include <cmath>

namespace {

static std::mt19937 sched_mt(42);
static std::unordered_set<int> free_agents;
static std::unordered_set<int> free_tasks;

struct Assignment {
    int agent;
    int task;
    int cost;
};

struct AssignRecord {
    int agent;
    int task;
    int raw_dist;
};

// 旋转次数
inline int get_turn_count(int curr_ori, int from_loc, int to_loc, const SharedEnvironment* env)
{
    if (from_loc == to_loc) return 0;
    int diff = to_loc - from_loc;
    int target_ori;
    if (diff == 1)              target_ori = 0;
    else if (diff == -1)        target_ori = 2;
    else if (diff == env->cols) target_ori = 1;
    else if (diff == -env->cols) target_ori = 3;
    else return 0;
    int angle = std::abs(curr_ori - target_ori);
    if (angle > 2) angle = 4 - angle;
    return angle;
}

// 完成任务所有剩余 errand 的真实最短路总距离
inline int compute_task_dist(int agent_loc, const Task& task, SharedEnvironment* env)
{
    int dist = 0;
    int c_loc = agent_loc;
    for (int j = task.idx_next_loc; j < (int)task.locations.size(); j++) {
        dist += DefaultPlanner::get_h(env, c_loc, task.locations[j]);
        c_loc = task.locations[j];
    }
    return dist;
}

// 拥堵热力图（5x5粗粒度方块）
void build_congestion_map(
    SharedEnvironment* env,
    std::unordered_map<int, int>& cong_map,
    int block_size = 5)
{
    cong_map.clear();
    int cols_blocks = env->cols / block_size + 1;
    for (int i = 0; i < env->num_of_agents; i++) {
        if (i < (int)env->goal_locations.size() && !env->goal_locations[i].empty()) {
            int goal = env->goal_locations[i].front().first;
            int block = (goal / env->cols) / block_size * cols_blocks
                      + (goal % env->cols) / block_size;
            cong_map[block]++;
        }
    }
}

inline int get_congestion(int loc, SharedEnvironment* env,
                          const std::unordered_map<int, int>& cong_map,
                          int block_size = 5)
{
    int cols_blocks = env->cols / block_size + 1;
    int block = (loc / env->cols) / block_size * cols_blocks
              + (loc % env->cols) / block_size;
    auto it = cong_map.find(block);
    return it != cong_map.end() ? it->second : 0;
}

void clean_free_tasks(SharedEnvironment* env)
{
    for (auto it = free_tasks.begin(); it != free_tasks.end(); ) {
        auto t_it = env->task_pool.find(*it);
        if (t_it == env->task_pool.end()
            || t_it->second.agent_assigned != -1
            || t_it->second.is_finished()) {
            it = free_tasks.erase(it);
        } else {
            ++it;
        }
    }
}

void clean_free_agents(const std::vector<int>& proposed_schedule)
{
    for (auto it = free_agents.begin(); it != free_agents.end(); ) {
        if (proposed_schedule[*it] != -1) {
            it = free_agents.erase(it);
        } else {
            ++it;
        }
    }
}

} // anonymous namespace


void TaskScheduler::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit / 5 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;
    DefaultPlanner::schedule_initialize(limit, env);
}


void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    // ARCH-C R1: Auction-based task assignment
    // Each agent bids on tasks. The task goes to the highest bidder.
    // Bid = eagerness score = (cost_benefit / distance) * urgency_multiplier
    // This is fundamentally different from greedy which picks lowest cost.
    // Greedy: "which agent should do this task?" (agent-centric)
    // Auction: "which task should this agent do?" (task-centric, reversed)

    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    clean_free_tasks(env);
    clean_free_agents(proposed_schedule);

    if (free_agents.empty() || free_tasks.empty()) {
        for (int ag : free_agents) proposed_schedule[ag] = -1;
        free_agents.clear();
        return;
    }

    int n_free = (int)free_agents.size();
    int n_tasks = (int)free_tasks.size();
    int pairs = n_free * n_tasks;

    bool do_sampling = (pairs > 50000);
    int max_pairs_to_eval = do_sampling ? 20000 : pairs;

    int sched_pct = (pairs > 10000) ? 25 : 20;
    int limit = time_limit * sched_pct / 100 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;
    if (limit > time_limit * 35 / 100) limit = time_limit * 35 / 100;

    TimePoint time_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    // 拥堵热力图
    std::unordered_map<int, int> cong_map;
    build_congestion_map(env, cong_map);

    // ====================================================================
    // Auction Phase: Each agent bids on its best task
    // Each agent computes the "eagerness" to do each candidate task
    // Higher eagerness = more likely to get the task
    // ====================================================================
    // Eagerness = base_eagerness / (dist + 1) * urgency
    // base_eagerness = reward for taking a task = wait_time + remaining_waypoints * 10
    // dist = path distance to task
    // urgency = how old/important the task is

    std::vector<Assignment> bids; // (agent, best_task, eagerness)

    const int K = std::min(16, n_tasks); // R15: increase from 12 to 16
    std::vector<std::pair<int,int>> task_dists;

    for (int ag : free_agents) {
        if (std::chrono::steady_clock::now() > time_end) break;

        int agent_loc = env->curr_states.at(ag).location;
        int agent_ori = env->curr_states.at(ag).orientation;

        // R120: K selection prefers low-congestion areas even if slightly farther
        // Use composite score: md + cong*5 (congestion penalty during selection)
        task_dists.clear();
        for (int t_id : free_tasks) {
            auto t_it = env->task_pool.find(t_id);
            if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1 || t_it->second.is_finished()) continue;
            int first_loc = t_it->second.locations.at(t_it->second.idx_next_loc);
            int md = DefaultPlanner::manhattanDistance(agent_loc, first_loc, env);
            int cong = get_congestion(first_loc, env, cong_map);
            int composite = md + cong * 7; // R129: try cong*7
            task_dists.push_back({composite, t_id});
        }

        if ((int)task_dists.size() > K) {
            std::nth_element(task_dists.begin(), task_dists.begin() + K, task_dists.end());
            task_dists.resize(K);
        }

        int best_task = -1;
        int best_eagerness = -999999;
        int best_dist = INT_MAX;

        for (auto& [md, t_id] : task_dists) {
            auto t_it = env->task_pool.find(t_id);
            if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1) continue;
            auto& task = t_it->second;

            int first_loc = task.locations.at(task.idx_next_loc);
            int dist = compute_task_dist(agent_loc, task, env);
            int turn = get_turn_count(agent_ori, agent_loc, first_loc, env);
            int cong = get_congestion(first_loc, env, cong_map);
            int wait = std::max(0, env->curr_timestep - task.t_revealed);
            int remaining = (int)task.locations.size() - task.idx_next_loc;

            // ARCH-C R83: higher penalty for high congestion (only >5 gets extra penalty)
            int log_dist = (int)(std::log(1.0 + dist) * 10.0); // scaled log
            int ratio = (remaining * 100 + wait * 16) * 1000 / (log_dist + 1 + cong * 7); // R129: try cong*7
            int cong_penalty = (cong > 3) ? (cong - 3) * 60 : 0; // R131: increase penalty 50->60
            // Extra penalty for very high congestion (>5)
            if (cong > 5) cong_penalty += (cong - 5) * 36; // R131: increase extra penalty 30->36
            int eagerness = ratio - turn * 2 - cong_penalty;

            // R101: distance tie-break — prefer closer task when eagerness similar
            // This helps agents complete tasks faster, reducing makespan
            if (eagerness > best_eagerness || 
                (eagerness == best_eagerness && dist < best_dist)) {
                best_eagerness = eagerness;
                best_task = t_id;
                best_dist = dist;
            }
        }

        if (best_task != -1) {
            bids.push_back({ag, best_task, best_eagerness});
        }
    }

    // ====================================================================
    // Allocation Phase: Each task goes to the highest bidder
    // Sort bids by eagerness (descending), allocate greedily
    // ====================================================================
    std::sort(bids.begin(), bids.end(),
        [](const Assignment& a, const Assignment& b) { return a.cost > b.cost; });

    std::vector<AssignRecord> this_round;
    std::unordered_set<int> assigned_tasks;
    std::unordered_set<int> assigned_agents;

    for (auto& b : bids) {
        if (assigned_tasks.count(b.task) || assigned_agents.count(b.agent)) continue;
        auto t_it = env->task_pool.find(b.task);
        if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1) continue;

        proposed_schedule[b.agent] = b.task;
        assigned_tasks.insert(b.task);
        assigned_agents.insert(b.agent);
        env->task_pool[b.task].agent_assigned = b.agent;

        int raw_dist = compute_task_dist(env->curr_states[b.agent].location, t_it->second, env);
        this_round.push_back({b.agent, b.task, raw_dist});
    }

    // 清理 free 集合
    for (auto& rec : this_round) {
        free_agents.erase(rec.agent);
        free_tasks.erase(rec.task);
    }
    for (int ag : free_agents) {
        proposed_schedule[ag] = -1;
    }

    // ====================================================================
    // Swap optimization phase (same as R7)
    // ====================================================================
    if (this_round.size() >= 2) {
        int max_swaps = std::min((int)this_round.size() * 5, 800);
        if (env->num_of_agents > 1000) max_swaps = std::min(max_swaps, 200);

        std::uniform_int_distribution<int> rand_idx(0, (int)this_round.size() - 1);
        TimePoint swap_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

        for (int iter = 0; iter < max_swaps; iter++) {
            if (std::chrono::steady_clock::now() > swap_end) break;

            int i1 = rand_idx(sched_mt);
            int i2 = rand_idx(sched_mt);
            if (i1 == i2) continue;

            auto& r1 = this_round[i1];
            auto& r2 = this_round[i2];

            int curr_cost = r1.raw_dist + r2.raw_dist;

            auto it1 = env->task_pool.find(r1.task);
            auto it2 = env->task_pool.find(r2.task);
            if (it1 == env->task_pool.end() || it2 == env->task_pool.end()) continue;

            int swap_dist1 = compute_task_dist(env->curr_states[r1.agent].location, it2->second, env);
            int swap_dist2 = compute_task_dist(env->curr_states[r2.agent].location, it1->second, env);
            int swap_cost = swap_dist1 + swap_dist2;

            if (swap_cost < curr_cost - 1) {
                proposed_schedule[r1.agent] = r2.task;
                proposed_schedule[r2.agent] = r1.task;
                env->task_pool[r1.task].agent_assigned = r2.agent;
                env->task_pool[r2.task].agent_assigned = r1.agent;
                r1.raw_dist = swap_dist1;
                r2.raw_dist = swap_dist2;
                std::swap(r1.task, r2.task);
            }
        }
    }
}
