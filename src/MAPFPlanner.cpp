#include <random>
#include "Entry.h"

// default planner 的头文件
#include "planner.h"
#include "const.h"

// ==================== MAPFPlanner::initialize ====================
// 预处理阶段：初始化 default planner 的所有数据结构
// 包括：启发式表、TrajLNS（轨迹+流优化）、PIBT 数据结构、优先级
//
// 改进：更精确的时间计算
// - 用 microseconds（不是 milliseconds）计算已用时间，减少误差
// - 这确保 planner 不会因为时间计算不精确而超时
// 灵感来自 Kitty Knight：他们也用 microseconds 来精确控制时间
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // 用剩余的预处理时间（减去 scheduler 已经用掉的）初始化 planner
    // - PLANNER_TIMELIMIT_TOLERANCE 是安全余量（10ms）
    // - 用 microseconds 计算已过时间，比 milliseconds 精确1000倍
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = preprocess_time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;

    DefaultPlanner::initialize(limit, env);
}

// ==================== MAPFPlanner::plan ====================
// 每个 planning episode 被调用一次
// 核心工作交给 DefaultPlanner::plan，它做了三件事：
//   1. 更新引导路径（guide path）+ Frank-Wolfe 流优化
//   2. 多步 PIBT 规划（按优先级决定每个机器人的下一步）
//   3. 动作后处理（处理旋转动作、移动检查）
//
// 我们的控制层主要做：
// - 精确的时间管理（把尽可能多的时间给 default planner）
// - 正确计算需要规划的步数
//
// 为什么不改 default planner 内部？
// - default planner 的 Flow+PIBT 组合已经很强大
// - No Man Sky 冠军队的 planner 也是类似的架构（引导路径 + PIBT）
// - 真正拉开差距的是 scheduler 的质量（我们已经在 TaskScheduler 中大幅改进）
void MAPFPlanner::plan(int time_limit, Plan & plan) 
{
    // 精确计算剩余时间
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;

    // 计算本次需要规划多少步
    // 公式：min_planner_communication_time / (action_time * max_counter) + 1
    // 
    // 例子：planCommTime=1000ms, action_time=1ms, max_counter=10
    //   → 1000 / (1 * 10) + 1 = 101 步
    //
    // 含义：两次 planning 调用之间有 1000ms，每步执行需要 10 个 tick（1ms/tick），
    //        所以需要 100 步动作来覆盖这段时间，+1 是安全余量
    //
    // 为什么不给更多步？
    // - 更多步 = 更长的规划时间，可能超时
    // - 更少步 = 动作不够，机器人到下轮 planning 前会原地等待
    // - 这个公式刚好平衡两者
    int min_plan_steps = std::max(1, (int)(env->min_planner_communication_time 
                       / (env->action_time * env->max_counter)));

    DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
}
