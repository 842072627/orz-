// MAPFPlanner.cpp — ORZ-only
#include <Entry.h>
#include <algorithm>
#include <iostream>
#include <random>
#include "planner.h"
#include "const.h"
#include "../optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.h"
#include "RuntimeConfig.h"

static optsvr::GameDirectionalPlanner* g_game_directional_planner_ = nullptr;

MAPFPlanner::~MAPFPlanner() {
    delete g_game_directional_planner_;
}

void MAPFPlanner::initialize(int preprocess_time_limit) {
    if (optsvr::cfg().planner_mode != 16) return;
    if (g_game_directional_planner_) {
        delete g_game_directional_planner_;
        g_game_directional_planner_ = nullptr;
    }
    int limit = preprocess_time_limit / 2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    g_game_directional_planner_ = new optsvr::GameDirectionalPlanner();
    g_game_directional_planner_->initialize(limit, env);
}

void MAPFPlanner::plan(int time_limit, Plan& plan) {
    if (optsvr::cfg().planner_mode == 16 && g_game_directional_planner_) {
        int limit = time_limit / 2 - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;
        g_game_directional_planner_->plan(limit, plan.actions, env, 6);
    }
}
