#include <algorithm>
#include <iostream>
#include <random>
#include <Entry.h>

#include "planner.h"
#include "const.h"

// TestUsefulFlowDecay planner (planner_mode=11, warehouse fullfill)
#include "optsvr/planner/TestUsefulFlowDecay/planner.h"
#include "RuntimeConfig.h"

static optsvr::TestUsefulFlowDecayPlanner* g_test_useful_flow_decay_planner_ = nullptr;

namespace {

void log_planned_action_steps(const SharedEnvironment* env,
                              const Plan& plan,
                              int planner_mode,
                              int requested_steps,
                              int limit_ms)
{
    if (env == nullptr) return;

    int min_steps = requested_steps;
    int max_steps = 0;
    int empty_agents = 0;

    if (plan.actions.empty()) {
        min_steps = 0;
    } else {
        min_steps = static_cast<int>(plan.actions.front().size());
        for (const auto& agent_actions : plan.actions) {
            const int steps = static_cast<int>(agent_actions.size());
            min_steps = std::min(min_steps, steps);
            max_steps = std::max(max_steps, steps);
            if (steps == 0) empty_agents++;
        }
    }

    std::cerr << "[MAPFPlanner] mode=" << planner_mode
              << " actions=" << min_steps << "-" << max_steps << "/" << requested_steps;
    if (empty_agents > 0) std::cerr << " empty=" << empty_agents;
    std::cerr << " limit_ms=" << limit_ms << "\n";
}

} // namespace

MAPFPlanner::~MAPFPlanner()
{
    delete g_test_useful_flow_decay_planner_;
    g_test_useful_flow_decay_planner_ = nullptr;
    delete env;
}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    if (optsvr::cfg().skip_test) return;

    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    DefaultPlanner::initialize(limit, env);

    if (optsvr::cfg().planner_mode == 11) {
        g_test_useful_flow_decay_planner_ = new optsvr::TestUsefulFlowDecayPlanner();
        g_test_useful_flow_decay_planner_->initialize(limit, env);
    }
}

void MAPFPlanner::plan(int time_limit, Plan & plan)
{
    if (optsvr::cfg().skip_test) return;

    int limit = time_limit - std::chrono::duration_cast<milliseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time / (env->action_time * env->max_counter)) + 1;

    if (g_test_useful_flow_decay_planner_ && optsvr::cfg().planner_mode == 11) {
        g_test_useful_flow_decay_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    }

    log_planned_action_steps(env, plan, optsvr::cfg().planner_mode, min_plan_steps, limit);
}
