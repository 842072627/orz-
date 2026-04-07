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

// ==================== 静态状态（跨时间步保持） ====================
// 原因：new_freeagents / new_tasks 每轮只给增量，必须累积
static std::mt19937 sched_mt(42);
static std::unordered_set<int> free_agents;  // 累积的空闲机器人
static std::unordered_set<int> free_tasks;   // 累积的未分配任务

// ==================== 数据结构 ====================
// 候选分配：(机器人, 任务, 总代价)
struct Assignment {
    int agent;
    int task;
    int cost;
};

// 本轮新分配记录（用于迭代优化阶段的 swap）
struct AssignRecord {
    int agent;
    int task;
    int raw_dist;  // 纯路径距离（不含附加惩罚），swap 时快速评估
};

// ==================== 辅助函数 ====================

// 计算从当前朝向旋转到目标方向需要的旋转次数
// 原因：在 MAPF-T 模型中，一次旋转 = max_counter 个 tick（默认10），
//        和前进一步耗时相同，所以旋转是很大的代价，必须计入调度成本
// 来自 RAPID 的启发：他们在 HeuristicTable 中考虑了朝向
inline int get_turn_count(int curr_ori, int from_loc, int to_loc, const SharedEnvironment* env)
{
    if (from_loc == to_loc) return 0;  // 不动不需要转
    
    // 根据位移推断需要朝向哪个方向
    int diff = to_loc - from_loc;
    int target_ori;
    if (diff == 1)             target_ori = 0;  // 东 (→)
    else if (diff == -1)       target_ori = 2;  // 西 (←)
    else if (diff == env->cols) target_ori = 1;  // 南 (↓)
    else if (diff == -env->cols) target_ori = 3; // 北 (↑)
    else return 0;  // 不是相邻格子，无法直接判断朝向

    // 计算最小旋转次数（0/1/2次）
    int angle = std::abs(curr_ori - target_ori);
    if (angle > 2) angle = 4 - angle;  // 反方向只需转2次（不是3次）
    return angle;
}

// 计算机器人完成一个任务所有剩余 errand 的真实最短路径总距离
// get_h 用的是 Dijkstra 预计算的启发式表，O(1)查询，考虑障碍物
inline int compute_task_dist(int agent_loc, const Task& task, SharedEnvironment* env)
{
    int dist = 0;
    int c_loc = agent_loc;
    for (int j = task.idx_next_loc; j < (int)task.locations.size(); j++) {
        dist += DefaultPlanner::get_h(env, c_loc, task.locations[j]);
        c_loc = task.locations[j];  // 顺路：走完一个 errand 后从那出发去下一个
    }
    return dist;
}

// 构建拥堵热力图（粗粒度：5x5方块）
// 原因：来自 RAPID 的核心洞见——很多机器人去同一区域会造成路径冲突，
//        调度时应该惩罚"热门区域"的任务，分散机器人
// 实现：将地图分成 5x5 的小块，统计每块有多少机器人的目标在那里
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

// 查询某个位置的拥堵值
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

// 清理 free_tasks 中已经分配/完成的任务
// 必须每次调用时清理，因为 task_pool 的状态会被系统更新
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

// 清理 free_agents 中已有任务的机器人
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


// ==================== initialize ====================
// 预处理阶段：只做启发式表预计算，极少的调度器初始化
// 原因：预处理时间很宝贵，应该大部分给 planner 预计算流优化数据
void TaskScheduler::initialize(int preprocess_time_limit)
{
    // 只给 20% 时间给 scheduler（默认给50%，你的版本给30%，这里进一步压缩到20%）
    // 原因：scheduler 的工作相对简单（只是匹配），planner 的启发式表和
    //        Flow 优化更耗时间且对结果影响更大
    int limit = preprocess_time_limit / 5 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;
    DefaultPlanner::schedule_initialize(limit, env);
}


// ==================== plan ====================
// 核心：三阶段调度——贪心匹配 → 迭代优化 swap → 收尾
// 灵感来源：
//   - No Man Sky 的 LNS（大邻域搜索）：先贪心初始解，再迭代改进
//   - RAPID 的 map_weights：拥堵感知的代价函数
//   - Kitty Knight 的自定义 scheduler：更精细的匹配
void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    // ===== 时间管理 =====
    // 动态分配：小问题给 scheduler 少一点时间，大问题多一点
    // 原因：小问题贪心就够了，swap 浪费时间；大问题需要更多 swap 迭代
    int pairs_estimate = (int)(free_agents.size() + env->new_freeagents.size())
                       * (int)(free_tasks.size() + env->new_tasks.size());
    int sched_pct = (pairs_estimate > 5000) ? 25 : 18;  // 18%-25%
    int limit = time_limit * sched_pct / 100 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;
    // 最多不超过 35%，给 planner 留足时间
    if (limit > time_limit * 35 / 100) limit = time_limit * 35 / 100;

    // 60% 的调度时间给贪心，40% 给 swap 优化
    int greedy_ms = limit * 60 / 100;
    int total_sched_ms = limit;
    TimePoint greedy_end  = std::chrono::steady_clock::now() + std::chrono::milliseconds(greedy_ms);
    TimePoint swap_end    = std::chrono::steady_clock::now() + std::chrono::milliseconds(total_sched_ms);

    // ===== 累积增量信息 =====
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    // 清理过期条目（上一轮可能已被系统/自己处理了）
    clean_free_tasks(env);
    clean_free_agents(proposed_schedule);

    // 无事可做，直接返回
    if (free_agents.empty() || free_tasks.empty()) {
        for (int ag : free_agents) proposed_schedule[ag] = -1;
        free_agents.clear();
        return;
    }

    // ===== 构建拥堵热力图 =====
    // 原因：如果很多机器人都去同一片区域，路会很堵
    // 来自 RAPID 的核心思路：用预计算的拥堵信息调整调度
    std::unordered_map<int, int> cong_map;
    build_congestion_map(env, cong_map);

    // ====================================================================
    // 第一阶段：构建所有 (机器人, 任务) 候选对，计算增强代价
    // ====================================================================
    // 代价 = 路径距离 + 旋转代价 + 拥堵惩罚 - 饥饿奖励 + 残余工作惩罚
    // 每一项都有明确的物理意义，下面逐一说明
    std::vector<Assignment> assignments;

    for (int t_id : free_tasks) {
        if (std::chrono::steady_clock::now() > greedy_end) break;
        auto t_it = env->task_pool.find(t_id);
        if (t_it == env->task_pool.end()) continue;
        auto& task = t_it->second;
        if (task.agent_assigned != -1 || task.is_finished()) continue;

        // 任务的第一个未完成 errand 位置
        int first_loc = task.locations.at(task.idx_next_loc);

        // 拥堵值：有多少机器人的目标在这个区域附近
        int cong = get_congestion(first_loc, env, cong_map);

        // 任务等待了多久（用于饥饿惩罚）
        int wait = std::max(0, env->curr_timestep - task.t_revealed);

        // 任务还剩多少个 errand
        int remaining = (int)task.locations.size() - task.idx_next_loc;

        for (int ag : free_agents) {
            if (std::chrono::steady_clock::now() > greedy_end) break;

            int agent_loc = env->curr_states.at(ag).location;
            int agent_ori = env->curr_states.at(ag).orientation;

            // --- 1. 路径距离：完成所有剩余 errand 的真实最短路总长 ---
            // 这是最核心的代价分量，用 Dijkstra 启发式表 O(1) 查询
            int dist = compute_task_dist(agent_loc, task, env);

            // --- 2. 旋转代价 ---
            // 机器人需要先转向才能朝目标走。每转一次 = max_counter tick
            // 如果机器人朝东，但第一个 errand 在北边，需要转1次（90°）
            // 默认 scheduler 完全不考虑这个，造成巨大浪费
            int turn = get_turn_count(agent_ori, agent_loc, first_loc, env);

            // --- 3. 拥堵惩罚 ---
            // 目标区域已经有 cong 个机器人往那里走了
            // 再派一个去会更堵，所以加惩罚
            // 系数3：经过调试，3是个平衡值（太小没效果，太大过度回避）
            int cong_penalty = cong * 3;

            // --- 4. 饥饿奖励（负值 = 减少代价 = 优先分配） ---
            // 等了很久还没被分配的任务应该更优先
            // 除以3：不要让等待时间完全压倒距离
            int starve_bonus = -wait / 3;

            // --- 5. 残余工作惩罚 ---
            // 剩余 errand 多的任务会占用机器人更久
            // 轻微惩罚长任务，让短任务先完成，释放机器人
            int future_penalty = remaining / 2;

            // --- 总代价 ---
            int cost = dist + turn + cong_penalty + starve_bonus + future_penalty;
            assignments.push_back({ag, t_id, cost});
        }
    }

    // ====================================================================
    // 第二阶段：全局贪心匹配
    // ====================================================================
    // 按代价排序，从最优匹配开始贪心选择
    // 比默认 scheduler 好在：默认是"每个机器人找最近任务"（以机器人为中心），
    // 我们是"全局排序后最优匹配"（以系统为中心），效果类似匈牙利算法的贪心近似
    std::sort(assignments.begin(), assignments.end(),
        [](const Assignment& a, const Assignment& b) { return a.cost < b.cost; });

    std::vector<AssignRecord> this_round;  // 本轮新分配记录
    std::unordered_set<int> assigned_tasks;
    std::unordered_set<int> assigned_agents;

    for (auto& a : assignments) {
        if (std::chrono::steady_clock::now() > greedy_end) break;
        if (assigned_tasks.count(a.task) || assigned_agents.count(a.agent)) continue;

        // 二次确认任务确实未被分配（防止 race condition）
        auto t_it = env->task_pool.find(a.task);
        if (t_it == env->task_pool.end() || t_it->second.agent_assigned != -1) continue;

        // 执行分配
        proposed_schedule[a.agent] = a.task;
        assigned_tasks.insert(a.task);
        assigned_agents.insert(a.agent);
        env->task_pool[a.task].agent_assigned = a.agent;

        // 记录本轮分配（用于 swap 阶段）
        int raw_dist = compute_task_dist(env->curr_states[a.agent].location,
                                          t_it->second, env);
        this_round.push_back({a.agent, a.task, raw_dist});
    }

    // 清理 free 集合：已分配的移除
    for (auto& rec : this_round) {
        free_agents.erase(rec.agent);
        free_tasks.erase(rec.task);
    }
    // 未分配的空闲机器人标记为 -1
    for (int ag : free_agents) {
        proposed_schedule[ag] = -1;
    }

    // ====================================================================
    // 第三阶段：迭代优化 —— 随机两两交换（LNS 的简化版）
    // ====================================================================
    // 灵感来自 No Man Sky 的 LNS（Large Neighborhood Search）
    // 原理：贪心匹配可能不是全局最优，比如：
    //   A→任务1（距离5），B→任务2（距离6）= 总11
    //   但 A→任务2（距离3），B→任务1（距离4）= 总7 ← 更好！
    // 做法是随机选两个已分配的机器人，试交换它们的任务，如果总距离下降就接受
    // 这是 LNS 的 destroy=2, repair=exhaustive 的特例
    if (this_round.size() >= 2) {
        int max_swaps = std::min((int)this_round.size() * 10, 1500);
        std::uniform_int_distribution<int> rand_idx(0, (int)this_round.size() - 1);

        for (int iter = 0; iter < max_swaps; iter++) {
            if (std::chrono::steady_clock::now() > swap_end) break;

            int i1 = rand_idx(sched_mt);
            int i2 = rand_idx(sched_mt);
            if (i1 == i2) continue;

            auto& r1 = this_round[i1];
            auto& r2 = this_round[i2];

            // 当前总距离
            int curr_cost = r1.raw_dist + r2.raw_dist;

            // 交换后的总距离
            auto& task1 = env->task_pool[r1.task];
            auto& task2 = env->task_pool[r2.task];
            int swap_dist1 = compute_task_dist(env->curr_states[r1.agent].location, task2, env);
            int swap_dist2 = compute_task_dist(env->curr_states[r2.agent].location, task1, env);
            int swap_cost = swap_dist1 + swap_dist2;

            // 接受条件：交换后总距离减少超过阈值2
            // 阈值避免微小改进导致的振荡，也避免代价函数误差导致无效交换
            if (swap_cost < curr_cost - 2) {
                // 执行交换
                proposed_schedule[r1.agent] = r2.task;
                proposed_schedule[r2.agent] = r1.task;
                env->task_pool[r1.task].agent_assigned = r2.agent;
                env->task_pool[r2.task].agent_assigned = r1.agent;

                // 更新记录
                r1.raw_dist = swap_dist1;  // r1.agent 现在做 task2
                r2.raw_dist = swap_dist2;  // r2.agent 现在做 task1
                std::swap(r1.task, r2.task);
            }
        }
    }
}
