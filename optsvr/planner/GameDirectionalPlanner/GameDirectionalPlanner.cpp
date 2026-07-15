// optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.cpp
// Migrates IDEA/gamePlanner/src/my_planner as an isolated planner mode.

#include "GameDirectionalPlanner.h"

#include "config.h"
#include "RuntimeConfig.h"
#include "orz_highway.h"
#include "my_planner/my_planner.h"
#include "planner.h"

#include <iostream>

namespace optsvr {

void GameDirectionalPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    enabled_for_map_ = is_ORZ(env);
    initialized_ = false;

    if (!enabled_for_map_) {
        std::cerr << "[GameDirectionalPlanner] map is not ORZ/game highway; fallback to DefaultPlanner\n";
        return;
    }

    if (cfg().game_highway_enabled) ORZ_Highway::init();
    MyPlanner::initialize(preprocess_time_limit, env);
    initialized_ = true;
    std::cerr << "[GameDirectionalPlanner] initialized highway="
              << (cfg().game_highway_enabled ? "hard" : "off")
              << " heuristic="
              << (cfg().game_guide_path_enabled ? "guide-flow" : "directed-bfs")
              << "\n";
}

void GameDirectionalPlanner::plan(int time_limit,
                                  std::vector<std::vector<Action>>& actions,
                                  SharedEnvironment* env,
                                  int num_steps) {
    if (!enabled_for_map_ || !initialized_) {
        DefaultPlanner::plan(time_limit, actions, env, num_steps);
        return;
    }

    MyPlanner::plan(time_limit, actions, env, num_steps);
}

} // namespace optsvr
