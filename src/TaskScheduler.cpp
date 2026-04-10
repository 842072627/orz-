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
    int throughput_score; // ARCH-B: higher is better
};

// 旋转次数：从当前朝向转到面向 to_loc 的方向需要几次旋转
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
    // 20% 给 scheduler，80% 给 planner
    int limit = preprocess_time_limit / 5 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;
    DefaultPlanner::schedule_initialize(limit, env);
}


void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    // ===== 累积增量信息 =====
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // 清理过期条目
    clean_free_tasks(env);
    clean_free_agents(proposed_schedule);

    if (free_agents.empty() || free_tasks.empty()) {
        for (int ag : free_agents) proposed_schedule[ag] = -1;
        free_agents.clear();
        return;
    }

    // ===== 时间管理 =====
    int n_free = (int)free_agents.size();
    int n_tasks = (int)free_tasks.size();
    int pairs = n_free * n_tasks;

    // 大地图限制：如果候选对太多，只采样一部分
    // 原因：warehouse_large_5000 上可能有上千空闲机器人×上千任务=百万对
    //        全部计算代价太大，采样足够得到好的匹配
    bool do_sampling = (pairs > 50000);
    int max_pairs_to_eval = do_sampling ? 20000 : pairs;

    int sched_pct = (pairs > 10000) ? 25 : 20;
    int limit = time_limit * sched_pct / 100 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;
    if (limit > time_limit * 35 / 100) limit = time_limit * 35 / 100;

    int greedy_ms = limit * 70 / 100;
    TimePoint greedy_end = std::chrono::steady_clock::now() + std::chrono::milliseconds(greedy_ms);
    TimePoint swap_end   = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);

    // ===== 拥堵热力图 =====
    std::unordered_map<int, int> cong_map;
    build_congestion_map(env, cong_map);

    // ====================================================================
    // 第一阶段：构建候选对 + 计算增强代价
    // ====================================================================
    std::vector<Assignment> assignments;

    if (do_sampling) {
        // 大规模模式：每个空闲机器人只评估最近的 K 个任务
        // 用曼哈顿距离快速筛选，避免对每个对都调用 get_h
        const int K = std::min(12, n_tasks);
        std::vector<std::pair<int,int>> task_dists; // (manhattan_dist, task_id)

        for (int ag : free_agents) {
            if (std::chrono::steady_clock::now() > greedy_end) break;
            int agent_loc = env->curr_states.at(ag).location;
            int agent_ori = env->curr_states.at(ag).orientation;

            // 快速用曼哈顿距离筛选最近的 K 个任务
            task_dists.clear();
            for (int t_id : free_tasks) {
                auto t_it = env->task_pool.find(t_id);
                if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1 || t_it->second.is_finished()) continue;
                int first_loc = t_it->second.locations.at(t_it->second.idx_next_loc);
                int md = DefaultPlanner::manhattanDistance(agent_loc, first_loc, env);
                task_dists.push_back({md, t_id});
            }
            // 部分排序：只取前 K 个
            if ((int)task_dists.size() > K) {
                std::nth_element(task_dists.begin(), task_dists.begin() + K, task_dists.end());
                task_dists.resize(K);
            }

            for (auto& [md, t_id] : task_dists) {
                if (std::chrono::steady_clock::now() > greedy_end) break;
                auto t_it = env->task_pool.find(t_id);
                if (t_it == env->task_pool.end()) continue;
                auto& task = t_it->second;
                if (task.agent_assigned != -1 || task.is_finished()) continue;

                int first_loc = task.locations.at(task.idx_next_loc);
                int dist = compute_task_dist(agent_loc, task, env);
                int turn = get_turn_count(agent_ori, agent_loc, first_loc, env);
                int cong = get_congestion(first_loc, env, cong_map);
                int wait = std::max(0, env->curr_timestep - task.t_revealed);
                int remaining = (int)task.locations.size() - task.idx_next_loc;

                // ARCH-B: Throughput-First scoring
                int score = wait * 50 + remaining * 150 - dist - turn * 2 - cong * 30;
                assignments.push_back({ag, t_id, -score});
            }
        }
    } else {
        // 小规模模式：评估所有对
        for (int t_id : free_tasks) {
            if (std::chrono::steady_clock::now() > greedy_end) break;
            auto t_it = env->task_pool.find(t_id);
            if (t_it == env->task_pool.end()) continue;
            auto& task = t_it->second;
            if (task.agent_assigned != -1 || task.is_finished()) continue;

            int first_loc = task.locations.at(task.idx_next_loc);
            int cong = get_congestion(first_loc, env, cong_map);
            int wait = std::max(0, env->curr_timestep - task.t_revealed);
            int remaining = (int)task.locations.size() - task.idx_next_loc;

            for (int ag : free_agents) {
                if (std::chrono::steady_clock::now() > greedy_end) break;
                int agent_loc = env->curr_states.at(ag).location;
                int agent_ori = env->curr_states.at(ag).orientation;

                int dist = compute_task_dist(agent_loc, task, env);
                int turn = get_turn_count(agent_ori, agent_loc, first_loc, env);

                // ARCH-B: Throughput-First scoring
                int score = wait * 50 + remaining * 150 - dist - turn * 2 - cong * 30;
                assignments.push_back({ag, t_id, -score});
            }
        }
    }

    // ====================================================================
    // 第二阶段：贪心匹配
    // ====================================================================
    std::sort(assignments.begin(), assignments.end(),
        [](const Assignment& a, const Assignment& b) { return a.cost < b.cost; });

    std::vector<AssignRecord> this_round;
    std::unordered_set<int> assigned_tasks;
    std::unordered_set<int> assigned_agents;

    for (auto& a : assignments) {
        if (assigned_tasks.count(a.task) || assigned_agents.count(a.agent)) continue;
        auto t_it = env->task_pool.find(a.task);
        if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1) continue;

        proposed_schedule[a.agent] = a.task;
        assigned_tasks.insert(a.task);
        assigned_agents.insert(a.agent);
        env->task_pool[a.task].agent_assigned = a.agent;

        // ARCH-B: compute throughput score at assignment time
        int agent_loc = env->curr_states.at(a.agent).location;
        int agent_ori = env->curr_states.at(a.agent).orientation;
        int first_loc = t_it->second.locations.at(t_it->second.idx_next_loc);
        int dist = compute_task_dist(agent_loc, t_it->second, env);
        int turn = get_turn_count(agent_ori, agent_loc, first_loc, env);
        int cong = get_congestion(first_loc, env, cong_map);
        int wait = std::max(0, env->curr_timestep - t_it->second.t_revealed);
        int remaining = (int)t_it->second.locations.size() - t_it->second.idx_next_loc;
        int tp_score = wait * 50 + remaining * 150 - dist - turn * 2 - cong * 30;
        this_round.push_back({a.agent, a.task, tp_score});
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
    // 第三阶段：迭代优化 swap（LNS 简化版）
    // ====================================================================
    // 只在分配数量合理时做 swap，大地图减少迭代次数
    if (this_round.size() >= 2) {
        int max_swaps = std::min((int)this_round.size() * 5, 800);
        // 大地图进一步减少
        if (env->num_of_agents > 1000) max_swaps = std::min(max_swaps, 200);
        
        std::uniform_int_distribution<int> rand_idx(0, (int)this_round.size() - 1);

        for (int iter = 0; iter < max_swaps; iter++) {
            if (std::chrono::steady_clock::now() > swap_end) break;

            int i1 = rand_idx(sched_mt);
            int i2 = rand_idx(sched_mt);
            if (i1 == i2) continue;

            auto& r1 = this_round[i1];
            auto& r2 = this_round[i2];

            // ARCH-B: swap if it improves total throughput score
            int curr_score = r1.throughput_score + r2.throughput_score;

            auto it1 = env->task_pool.find(r1.task);
            auto it2 = env->task_pool.find(r2.task);
            if (it1 == env->task_pool.end() || it2 == env->task_pool.end()) continue;

            int ag1_loc = env->curr_states[r1.agent].location;
            int ag1_ori = env->curr_states[r1.agent].orientation;
            int ag2_loc = env->curr_states[r2.agent].location;
            int ag2_ori = env->curr_states[r2.agent].orientation;
            int t1_first = it1->second.locations.at(it1->second.idx_next_loc);
            int t2_first = it2->second.locations.at(it2->second.idx_next_loc);
            int d1 = compute_task_dist(ag1_loc, it2->second, env);
            int t1 = get_turn_count(ag1_ori, ag1_loc, t2_first, env);
            int c1 = get_congestion(t2_first, env, cong_map);
            int w1 = std::max(0, env->curr_timestep - it2->second.t_revealed);
            int rem1 = (int)it2->second.locations.size() - it2->second.idx_next_loc;
            int swap_score1 = w1 * 50 + rem1 * 150 - d1 - t1 * 2 - c1 * 30;
            int d2 = compute_task_dist(ag2_loc, it1->second, env);
            int t2 = get_turn_count(ag2_ori, ag2_loc, t1_first, env);
            int c2 = get_congestion(t1_first, env, cong_map);
            int w2 = std::max(0, env->curr_timestep - it1->second.t_revealed);
            int rem2 = (int)it1->second.locations.size() - it1->second.idx_next_loc;
            int swap_score2 = w2 * 50 + rem2 * 150 - d2 - t2 * 2 - c2 * 30;
            int swap_score = swap_score1 + swap_score2;

            if (swap_score > curr_score + 50) { // require meaningful improvement
                proposed_schedule[r1.agent] = r2.task;
                proposed_schedule[r2.agent] = r1.task;
                env->task_pool[r1.task].agent_assigned = r2.agent;
                env->task_pool[r2.task].agent_assigned = r1.agent;
                r1.throughput_score = swap_score1;
                r2.throughput_score = swap_score2;
                std::swap(r1.task, r2.task);
            }
        }
    }
}
