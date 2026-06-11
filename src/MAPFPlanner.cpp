#include <Entry.h>

#include "planner.h"
#include "const.h"

// FlowDecay planner (optsvr)
#include "../optsvr/planner/FlowDecay/planner.h"
#include "RuntimeConfig.h"

static optsvr::OptPlanner* g_opt_planner_ = nullptr;

MAPFPlanner::~MAPFPlanner()
{
    delete g_opt_planner_;
    g_opt_planner_ = nullptr;
    delete env;
}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    if (optsvr::cfg().skip_test) return;

    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    DefaultPlanner::initialize(limit, env);

    if (optsvr::cfg().planner_mode == 1) {
        g_opt_planner_ = new optsvr::OptPlanner();
        g_opt_planner_->initialize(limit, env);
    }
}

void MAPFPlanner::plan(int time_limit, Plan & plan)
{
    if (optsvr::cfg().skip_test) return;

    int limit = time_limit - std::chrono::duration_cast<milliseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time / (env->action_time * env->max_counter)) + 1;

    if (g_opt_planner_ && optsvr::cfg().planner_mode == 1) {
        g_opt_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    }
}
