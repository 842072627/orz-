// optsvr/GreedyHeapMatcher.h — LORR24-style Greedy Heap Matcher
//
// 算法来源：LORR2024 冠军方案 SchedulerSolver
// 核心思想：
//   Phase 1 (rebuild): 为每个 free robot 构建按 score 排序的候选 task 列表
//                      支持多线程并行 + 跨时间步缓存（未更新的 robot 优先重建）
//   Phase 2 (lazy_solve): 全局 min-heap 贪心匹配
//                         堆顶 (score, robot, index)，pop 后检查 task 是否已被占用
//                         若已占用则 index++ 跳到下一个候选，push 回堆（lazy skip）
//   Phase 3 (lns_refine): 可选 SA-based 随机 swap 优化（默认关闭）
//
// 打分公式：内置 dist_weight * dist + task_length（LORR24 原始公式）
// 可选密度惩罚：+ density_weight * congestion(task_start)
// density_weight=0 时退化为原始公式（零开销）
#pragma once

#include "SharedEnv.h"
#include "TaskMatcher.h"   // AgentInfo, TaskInfo, MatchResult
#include "RuntimeConfig.h"
#include <vector>
#include <chrono>
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace optsvr {

class GreedyHeapMatcher {
public:
    // ============ 初始化 ============
    void set_env(SharedEnvironment* env) { env_ = env; }

    // 设置拥塞查询函数（由 OptScheduler 注入）
    // 函数签名：float(int location) -> 该位置区域的拥塞度
    void set_congestion_func(std::function<float(int)> func) {
        congestion_func_ = std::move(func);
    }

    // 设置当前时间步和总步数（供 age/completion 计算）
    void set_timestep_info(int curr_timestep, int total_steps) {
        curr_timestep_ = curr_timestep;
        total_steps_   = total_steps;
    }

    // Optional task-level additive cost used by schedulers that compute a
    // route-load penalty outside the matcher. Empty means no extra cost.
    void set_task_extra_costs(std::vector<float> costs);

    // Optional pair-level additive cost. The callback receives
    // (agent_location, task_index_in_current_match).
    void set_pair_extra_cost_func(std::function<float(int, int)> func);

    // ============ 核心入口 ============
    // 传入 free agents / free tasks 信息，在 deadline 前完成匹配
    // 内部自动拆分 rebuild / lazy_solve / lns_refine 三个阶段的时间预算
    //
    // reassign_pairs: reassign agents 的旧 (agent_id, task_id) 配对，
    //   Phase 1+2 只匹配 agents/tasks（free agents），Phase 3 LNS 将
    //   reassign_pairs 与 free agents 的匹配结果合并后一起做 SA swap。
    //   reassign_pairs 对应的 AgentInfo/TaskInfo 也需要传入（拼接在 agents/tasks 后面，
    //   或由调用方提供独立的 reassign_agents_info/reassign_tasks_info）。
    //   为简洁起见，这里要求调用方把 reassign 的 agent/task 追加到 agents/tasks 尾部，
    //   rebuild 和 solve 只处理前 n_free 个 agent，LNS 处理全部。
    // old_assignment: agent_id → old_task_id，用于 LNS bias
    // max_matches: Phase 2 最多确认的匹配数（-1=不限制，全部匹配）
    //   当 < agent 数时，堆保证全局 score 最低的 top-K 配对被选中
    MatchResult match(
        const std::vector<AgentInfo>& agents,
        const std::vector<TaskInfo>& tasks,
        std::chrono::steady_clock::time_point deadline,
        int n_free_agents = -1,
        const std::unordered_map<int, int>& old_assignment = {},
        int max_matches = -1);

    // 单独调用各阶段（高级用法，由 OptScheduler 按需分配时间预算）
    void prepare(
        const std::vector<AgentInfo>& agents,
        const std::vector<TaskInfo>& tasks);

    void rebuild_candidates(std::chrono::steady_clock::time_point deadline);

    MatchResult lazy_heap_solve(std::chrono::steady_clock::time_point deadline, int max_matches = -1);

    // 贪心 swap 优化：随机交换两个 agent 的 task，只接受改进
    MatchResult lns_refine(
        MatchResult initial,
        const std::vector<AgentInfo>& agents,
        const std::vector<TaskInfo>& tasks,
        std::chrono::steady_clock::time_point deadline,
        const std::unordered_map<int, int>& old_assignment = {});

private:
    SharedEnvironment* env_ = nullptr;

    // ---- 拥塞查询（由 OptScheduler 注入） ----
    std::function<float(int)> congestion_func_ = nullptr;

    // ---- 候选列表缓存（跨时间步复用） ----
    // candidates_[robot_idx] = sorted vector of (score, task_idx)
    // 按 score 升序排列，score 越低越优
    // 只有前 sort_k 个保证有序，其余保证 >= 第 sort_k 个
    std::vector<std::vector<std::pair<float, int>>> candidates_;

    // 每个 robot 上次更新候选列表的时间步（用于优先重建最旧的）
    std::vector<int> timestep_updated_;

    // ---- 当前轮次的 agents / tasks 快照指针 ----
    const std::vector<AgentInfo>* agents_ptr_ = nullptr;
    const std::vector<TaskInfo>*  tasks_ptr_  = nullptr;

    // ---- task 起点位置连续数组（cache-friendly） ----
    // 每轮 prepare() 时从 tasks_ptr_ 提取，避免间接寻址
    std::vector<int> task_start_locs_;

    // ---- task_length 缓存（每轮计算一次，所有 agent 共享） ----
    // task_lengths_[j] = task j 的总路径长度（errand 段距离之和）
    // 在 prepare() 中预计算，rebuild_single 直接查表
    std::vector<float> task_lengths_;
    bool task_lengths_valid_ = false;

    // ---- 外部注入的 task 级额外代价 ----
    std::vector<float> task_extra_costs_;

    // ---- 外部注入的 agent-task pair 级额外代价 ----
    std::function<float(int, int)> pair_extra_cost_func_ = nullptr;

    // ---- agent 位置连续数组（rebuild 多线程 cache-friendly） ----
    // 每轮 prepare() 时提取，让 rebuild 线程按连续块访问
    std::vector<int> agent_locs_;

    // ---- 缓存的配置参数（prepare() 时从 cfg() 读取一次） ----
    float cached_dist_weight_ = 1.0f;
    float cached_cong_weight_ = 0.0f;  // 拥塞惩罚权重（0=关闭）
    float cached_dist_pow_    = 1.0f;  // 距离指数（1.0=线性）
    float cached_age_weight_  = 0.0f;  // Task Age 饥饿度权重（0=关闭）
    float cached_comp_weight_ = 0.0f;  // Completion Potential 权重（0=关闭）
    int   cached_decay_interval_ = 0;  // dist weight 一次性衰减触发步数（0=关闭）
    float cached_decay_factor_ = 1.0f; // 触发后乘上的系数
    int   cached_decay_steps_ = 0;     // 本轮是否已触发一次性衰减
    int   curr_timestep_      = 0;     // 当前仿真步（由外部设置）
    int   total_steps_        = 5000;  // 总仿真步数

    // ---- task 起点拥塞缓存（每轮 prepare 预计算一次） ----
    std::vector<float> task_congestion_;

    // ---- task 下发时间缓存 ----
    std::vector<int> task_ages_;  // task_ages_[j] = curr_timestep - t_revealed

    // ---- 打分函数 ----
    // 内置打分：dist_weight * dist + task_length（使用缓存，高效无虚调用）
    float compute_score_fast(int agent_loc, int task_idx) const;

    // 计算单个 task 的自身长度（所有 errand 段距离之和）
    float compute_task_length(const TaskInfo& task) const;

    // ---- 单个 robot 的候选列表重建 ----
    void rebuild_single(int agent_idx);

    // ---- 线程内批量重建（连续块，cache-friendly） ----
    // 处理 order[begin..end) 范围内的 agent，检查 deadline
    // 返回实际重建的数量
    int rebuild_chunk(const std::vector<int>& order, int begin, int end,
                      std::chrono::steady_clock::time_point deadline);
};

} // namespace optsvr
