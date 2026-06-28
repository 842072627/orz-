#include <algorithm>
#include <iostream>
#include <random>
#include <Entry.h>

#include "planner.h"
#include "const.h"

// FlowDecay planner (optsvr/planner/FlowDecay/planner.h)
#include "../optsvr/planner/FlowDecay/planner.h"
// GameOpt planner (optsvr/planner/GameOpt/GameOptPlanner.h)
#include "../optsvr/planner/GameOpt/GameOptPlanner.h"
// TurnAware planner (optsvr/planner/TurnAware/TurnAwarePlanner.h)
#include "../optsvr/planner/TurnAware/TurnAwarePlanner.h"
// EPIBT planner (optsvr/planner/EPIBT/EPIBTPlanner.h)
#include "../optsvr/planner/EPIBT/EPIBTPlanner.h"
// RandomFlowEPIBT planner (optsvr/planner/RandomFlowEPIBT/RandomFlowEPIBTPlanner.h)
#include "../optsvr/planner/RandomFlowEPIBT/RandomFlowEPIBTPlanner.h"
// IdleYield planner (optsvr/planner/IdleYield/IdleYieldPlanner.h)
#include "../optsvr/planner/IdleYield/IdleYieldPlanner.h"
// RoomPlanner (optsvr/planner/RoomPlanner/planner.h)
#include "../optsvr/planner/RoomPlanner/planner.h"
// RoomTidalPlanner (optsvr/planner/RoomTidalPlanner/RoomTidalPlanner.h)
#include "../optsvr/planner/RoomTidalPlanner/RoomTidalPlanner.h"
// MazeTidalPlanner (optsvr/planner/MazeTidalPlanner/MazeTidalPlanner.h)
#include "../optsvr/planner/MazeTidalPlanner/MazeTidalPlanner.h"
// TestUsefulFlowDecay planner (optsvr/planner/TestUsefulFlowDecay/planner.h)
#include "../optsvr/planner/TestUsefulFlowDecay/planner.h"
// IronFlowDecay planner (optsvr/planner/IronFlowDecay/planner.h)
#include "../optsvr/planner/IronFlowDecay/planner.h"
#include "RuntimeConfig.h"

static optsvr::GameOptPlanner* g_game_planner_ = nullptr;
static optsvr::TurnAwarePlanner* g_ta_planner_ = nullptr;
static optsvr::EPIBTPlanner* g_epibt_planner_ = nullptr;
static optsvr::RandomFlowEPIBTPlanner* g_random_flow_epibt_planner_ = nullptr;
static optsvr::IdleYieldPlanner* g_idle_yield_planner_ = nullptr;
static optsvr::RoomPlanner* g_room_planner_ = nullptr;
static optsvr::RoomTidalPlanner* g_tidal_planner_ = nullptr;
static optsvr::MazeTidalPlanner* g_maze_tidal_planner_ = nullptr;
static optsvr::TestUsefulFlowDecayPlanner* g_test_useful_flow_decay_planner_ = nullptr;
static optsvr::IronFlowDecayPlanner* g_iron_flow_decay_planner_ = nullptr;

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
    delete opt_planner_;
    delete g_game_planner_;
    g_game_planner_ = nullptr;
    delete g_ta_planner_;
    g_ta_planner_ = nullptr;
    delete g_epibt_planner_;
    g_epibt_planner_ = nullptr;
    delete g_random_flow_epibt_planner_;
    g_random_flow_epibt_planner_ = nullptr;
    delete g_idle_yield_planner_;
    g_idle_yield_planner_ = nullptr;
    delete g_room_planner_;
    g_room_planner_ = nullptr;
    delete g_tidal_planner_;
    g_tidal_planner_ = nullptr;
    delete g_maze_tidal_planner_;
    g_maze_tidal_planner_ = nullptr;
    delete g_test_useful_flow_decay_planner_;
    g_test_useful_flow_decay_planner_ = nullptr;
    delete g_iron_flow_decay_planner_;
    g_iron_flow_decay_planner_ = nullptr;
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
        opt_planner_ = new optsvr::OptPlanner();
        opt_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 2) {
        g_game_planner_ = new optsvr::GameOptPlanner();
        g_game_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 3) {
        g_ta_planner_ = new optsvr::TurnAwarePlanner();
        g_ta_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 4) {
        g_epibt_planner_ = new optsvr::EPIBTPlanner();
        g_epibt_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 10) {
        g_random_flow_epibt_planner_ = new optsvr::RandomFlowEPIBTPlanner();
        g_random_flow_epibt_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 5) {
        g_idle_yield_planner_ = new optsvr::IdleYieldPlanner();
        g_idle_yield_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 6) {
        g_room_planner_ = new optsvr::RoomPlanner();
        g_room_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 7) {
        g_tidal_planner_ = new optsvr::RoomTidalPlanner();
        g_tidal_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 8) {
        g_maze_tidal_planner_ = new optsvr::MazeTidalPlanner();
        g_maze_tidal_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 11) {
        g_test_useful_flow_decay_planner_ = new optsvr::TestUsefulFlowDecayPlanner();
        g_test_useful_flow_decay_planner_->initialize(limit, env);
    } else if (optsvr::cfg().planner_mode == 12) {
        g_iron_flow_decay_planner_ = new optsvr::IronFlowDecayPlanner();
        g_iron_flow_decay_planner_->initialize(limit, env);
    }
}

void MAPFPlanner::plan(int time_limit, Plan & plan)
{
    if (optsvr::cfg().skip_test) return;

    int limit = time_limit - std::chrono::duration_cast<milliseconds>(
        std::chrono::steady_clock::now() - env->plan_start_time).count()
        - DefaultPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time / (env->action_time * env->max_counter)) + 1;

    if (g_game_planner_ && optsvr::cfg().planner_mode == 2) {
        g_game_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_ta_planner_ && optsvr::cfg().planner_mode == 3) {
        g_ta_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_epibt_planner_ && optsvr::cfg().planner_mode == 4) {
        g_epibt_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_random_flow_epibt_planner_ && optsvr::cfg().planner_mode == 10) {
        g_random_flow_epibt_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_idle_yield_planner_ && optsvr::cfg().planner_mode == 5) {
        g_idle_yield_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_room_planner_ && optsvr::cfg().planner_mode == 6) {
        g_room_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_tidal_planner_ && optsvr::cfg().planner_mode == 7) {
        g_tidal_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_maze_tidal_planner_ && optsvr::cfg().planner_mode == 8) {
        g_maze_tidal_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_test_useful_flow_decay_planner_ && optsvr::cfg().planner_mode == 11) {
        g_test_useful_flow_decay_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (g_iron_flow_decay_planner_ && optsvr::cfg().planner_mode == 12) {
        g_iron_flow_decay_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else if (opt_planner_ && optsvr::cfg().planner_mode == 1) {
        opt_planner_->plan(limit, plan.actions, env, min_plan_steps);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    }

    log_planned_action_steps(env, plan, optsvr::cfg().planner_mode, min_plan_steps, limit);
}
