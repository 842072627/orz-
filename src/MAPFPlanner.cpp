// MAPFPlanner.cpp — Combined Track: ORZ用定制规划器，其他图用默认PIBT
#include <random>
#include <Entry.h>
#include "config.h"
#include "planner.h"
#include "const.h"

void MAPFPlanner::initialize(int preprocess_time_limit) {
    int limit = preprocess_time_limit - 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() 
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    
    if (has_custom_strategy(env)) {
        // ORZ: 定制规划器初始化
        // TODO: 实现ORZ专属规划器
        DefaultPlanner::initialize(limit, env);
    } else {
        DefaultPlanner::initialize(limit, env);
    }
}

void MAPFPlanner::plan(int time_limit, Plan & plan) {
    int limit = time_limit - 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() 
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    int min_plan_steps = (int)(env->min_planner_communication_time / 
        (env->action_time * env->max_counter)) + 1;
    
    if (has_custom_strategy(env)) {
        // ORZ: 定制规划器
        // TODO: 实现ORZ专属规划器（基于Kitty Knight代码）
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    }
}
