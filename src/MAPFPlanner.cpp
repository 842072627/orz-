#include <Entry.h>
#include "planner.h"
#include "const.h"
#include "../optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.h"
#include "RuntimeConfig.h"

static optsvr::GameDirectionalPlanner* g_game_directional_planner_ = nullptr;

MAPFPlanner::~MAPFPlanner() {}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    if (optsvr::cfg().skip_test) return;
    int limit = preprocess_time_limit;
    DefaultPlanner::initialize(limit, env);
    g_game_directional_planner_ = new optsvr::GameDirectionalPlanner();
    g_game_directional_planner_->initialize(limit, env);
}

void MAPFPlanner::plan(int time_limit, Plan& plan)
{
    if (optsvr::cfg().skip_test) return;
    int limit = time_limit;
    if (g_game_directional_planner_) {
        g_game_directional_planner_->plan(limit, plan.actions, env, 1);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, 1);
    }
}
