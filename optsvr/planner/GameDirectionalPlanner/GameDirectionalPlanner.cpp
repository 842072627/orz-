// optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.cpp
// Migrates IDEA/gamePlanner/src/my_planner as an isolated planner mode.

#include "GameDirectionalPlanner.h"

#include "config.h"
#include "orz_highway.h"
#include "my_planner/my_planner.h"
#include "planner.h"

#include <iostream>

namespace nz {

void GameDirectionalPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env) {
    enabled_for_map_ = is_ORZ(env);
    initialized_ = false;

    if (!enabled_for_map_) {
        std::cerr << "[GameDirectionalPlanner] map is not ORZ/game highway; fallback to DefaultPlanner\n";
        return;
    }

    ORZ_Highway::init();
    MyPlanner::initialize(preprocess_time_limit, env);
    initialized_ = true;
    std::cerr << "[GameDirectionalPlanner] initialized with ORZ highway directional constraints\n";
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

} // namespace nz
