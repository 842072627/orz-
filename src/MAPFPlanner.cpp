// MAPFPlanner.cpp — ORZ-only: only uses GameDirectionalPlanner
#include <Entry.h>
#include "Plan.h"
#include "planner.h"
#include "const.h"
#include "../optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.h"
#include "RuntimeConfig.h"

#include <chrono>

using namespace std::chrono;

static optsvr::GameDirectionalPlanner* g_game_directional_planner_ = nullptr;
static bool g_game_directional_planner_initialized_ = false;

MAPFPlanner::~MAPFPlanner()
{
    delete g_game_directional_planner_;
}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    if (optsvr::cfg().skip_test) return;

    int limit = preprocess_time_limit - duration_cast<milliseconds>(
        steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    if (optsvr::cfg().planner_mode == 16) {
        if (g_game_directional_planner_) {
            delete g_game_directional_planner_;
            g_game_directional_planner_ = nullptr;
        }
        g_game_directional_planner_ = new optsvr::GameDirectionalPlanner();
        g_game_directional_planner_->initialize(limit, env);
        g_game_directional_planner_initialized_ = true;
    }
}

void MAPFPlanner::plan(int time_limit, Plan& plan)
{
    if (optsvr::cfg().skip_test) return;

    int limit = time_limit - duration_cast<milliseconds>(
        steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time / (env->action_time * env->max_counter)) + 1;

    if (g_game_directional_planner_ && optsvr::cfg().planner_mode == 16) {
        g_game_directional_planner_->plan(limit, plan.actions, env, min_plan_steps);
    }
}
