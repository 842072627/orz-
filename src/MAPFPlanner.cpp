#include <random>
#include "Entry.h"

// default planner 的头文件
#include "planner.h"
#include "const.h"

// ==================== ARCH-G 实验 ====================
// 方向：基于拥堵感知的自适应规划步数
// 灵感来自 No_Man_Sky 的 adaptive planning
// 思路：
//   - 当 free agents 多时（高并发），减少规划步数提高响应速度
//   - 当 free agents 少时（低并发），增加规划步数减少规划开销
//
// ARCH-D S1 baseline: min_plan_steps = base * 1.2
// ARCH-G G1: 自适应调整
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

    // ARCH-G G1: 基于并发度的自适应规划步数
    // 
    // 基础步数 (与 S1 相同)
    int base_steps = std::max(1, (int)(env->min_planner_communication_time 
                       / (env->action_time * env->max_counter) * 1.2));
    
    // 计算当前并发度: free_agents / total_agents
    // env->new_freeagents 是新释放的 agents，env->num_of_agents 是总数
    int free_count = env->new_freeagents.size();
    int total_agents = env->num_of_agents;
    double concurrency_ratio = (total_agents > 0) ? (double)free_count / total_agents : 0.5;
    
    // 自适应调整因子:
    // - 高并发(congestion)时，减少步数(更快速响应)
    // - 低并发时，增加步数(减少规划频率)
    // concurrency_ratio 范围 [0, 1]
    // ratio > 0.3 → 高并发 → 乘以 0.9 (减少步数)
    // ratio <= 0.3 → 低并发 → 乘以 1.0 (保持不变)
    double adaptive_factor = (concurrency_ratio > 0.3) ? 0.9 : 1.0;
    
    int min_plan_steps = std::max(1, (int)(base_steps * adaptive_factor));
    
    // 调试输出
    // std::cout << "[ARCH-G G1] free=" << free_count << "/" << total_agents
    //           << " ratio=" << concurrency_ratio
    //           << " factor=" << adaptive_factor
    //           << " steps=" << min_plan_steps << std::endl;

    DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
}
