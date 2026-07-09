// MAPFPlanner.cpp — ORZ-only: only uses GameDirectionalPlanner
#include <Entry.h>
#include "Plan.h"
#include "planner.h"
#include "const.h"
#include "../optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.h"
#include "RuntimeConfig.h"

static optsvr::GameDirectionalPlanner g_game_directional_planner;

MAPFPlanner::~MAPFPlanner() {}

void MAPFPlanner::initialize(int preprocess_time_limit)
{
    if (optsvr::cfg().planner_mode == 16) {
        g_game_directional_planner.initialize(preprocess_time_limit, env);
    }
}

void MAPFPlanner::plan(int time_limit, Plan& plan)
{
    if (optsvr::cfg().planner_mode == 16) {
        g_game_directional_planner.plan(time_limit, plan.actions, env, 1);
    }
}
