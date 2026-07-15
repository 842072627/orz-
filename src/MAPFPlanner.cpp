#include "MAPFPlanner.h"
#include "GameDirectionalPlanner.h"
#include "RuntimeConfig.h"
#include "const.h"
#include "planner.h"

#include <algorithm>
#include <chrono>

namespace {
optsvr::GameDirectionalPlanner game_planner;
}

MAPFPlanner::~MAPFPlanner() = default;

void MAPFPlanner::initialize(int preprocess_time_limit) {
    if (optsvr::cfg().skip_test) return;
    const int limit = preprocess_time_limit -
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() -
        DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    DefaultPlanner::initialize(limit, env);
    game_planner.initialize(limit, env);
}

void MAPFPlanner::plan(int time_limit, Plan& plan) {
    if (optsvr::cfg().skip_test) return;
    const int limit = time_limit -
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - env->plan_start_time).count() -
        DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    const int action_ticks = std::max(1, env->action_time * env->max_counter);
    const int num_steps = env->min_planner_communication_time / action_ticks + 1;
    game_planner.plan(limit, plan.actions, env, num_steps);
}
