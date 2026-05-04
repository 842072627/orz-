// MAPFPlanner.cpp — Combined Track: ORZ用BFS增强规划器，其他图默认
#include <random>
#include <Entry.h>
#include "config.h"
#include "planner.h"
#include "const.h"
#include "strategy_bfs_full.h"

// ORZ自定义规划器: 用预计算BFS距离做更好的路径选择
void MAPFPlanner::initialize(int preprocess_time_limit) {
    int limit = preprocess_time_limit - 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() 
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    
    if (has_custom_strategy(env)) {
        // ORZ: 预计算BFS表（30分钟预算内）
        strategy_bfs_full::init();
        strategy_bfs_full::precompute(env);
    }
    // 使用默认PIBT规划器
    DefaultPlanner::initialize(limit, env);
}

void MAPFPlanner::plan(int time_limit, Plan & plan) {
    int limit = time_limit - 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() 
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    int min_plan_steps = (int)(env->min_planner_communication_time / 
        (env->action_time * env->max_counter)) + 1;
    
    // ORZ: 使用默认PIBT + BFS距离增强
    // （未来可替换为自定义PIBT）
    DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
}
