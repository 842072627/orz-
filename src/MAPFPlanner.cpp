#include <random>
#include "Entry.h"

// default planner 的头文件
#include "planner.h"
#include "const.h"

// ==================== ARCH-G 实验 ====================
// 方向：随机化PIBTrand种子，打破对称性
// 灵感来自 No_Man_Sky 的 EPlanner: 多线程多次采样 + 选最优
// 
// 关键发现：pibt.cpp 中使用 rand() 打破平局
// 我们在规划开始前设置基于时间的随机种子
//
// ARCH-D S1 baseline: srand(0) 每次相同
// ARCH-G G1: 基于时间设置随机种子
// ==================== MAPFPlanner::initialize ====================
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = preprocess_time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 10) limit = 10;

    DefaultPlanner::initialize(limit, env);
}

// ==================== MAPFPlanner::plan ====================
void MAPFPlanner::plan(int time_limit, Plan & plan) 
{
    int elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count();
    int limit = time_limit - elapsed_us / 1000 
              - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    if (limit < 5) limit = 5;

    // ========== ARCH-G G2: 增强随机化rand种子 ==========
    // 关键洞察：DefaultPlanner::initialize() 中 srand(0) 被设置为固定种子
    // 这导致每次运行的PIBTrand平局结果完全相同
    // 
    // 增强方案：使用更多熵源
    // - 时间戳的多个部分
    // - curr_timestep
    // - num_of_agents (区分不同规模的实例)
    auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    unsigned int random_seed = static_cast<unsigned int>(
        ((now >> 16) & 0xFFFF) ^  // 时间戳高16位
        ((now << 8) & 0xFFFF0000) ^  // 时间戳低16位移位
        (static_cast<unsigned int>(env->curr_timestep) * 31) ^  // 时间步
        (static_cast<unsigned int>(env->num_of_agents) * 17)  // agent数量
    );
    srand(random_seed);
    
    // 计算规划步数
    int base_steps = std::max(1, (int)(env->min_planner_communication_time 
                       / (env->action_time * env->max_counter) * 1.2));
    int min_plan_steps = std::max(1, (int)(base_steps));

    // 调试输出（注释掉以减少输出）
    // std::cout << "[ARCH-G G1] seed=" << random_seed 
    //           << " timestep=" << env->curr_timestep
    //           << " steps=" << min_plan_steps << std::endl;

    DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
}
