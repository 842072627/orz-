// optsvr/planner/GameDirectionalPlanner/GameDirectionalPlanner.h
// Wrapper for IDEA/gamePlanner's ORZ highway directional planner.
#pragma once

#include "SharedEnv.h"
#include "ActionModel.h"
#include <vector>

namespace optsvr {

class GameDirectionalPlanner {
public:
    void initialize(int preprocess_time_limit, SharedEnvironment* env);
    void plan(int time_limit, std::vector<std::vector<Action>>& actions,
              SharedEnvironment* env, int num_steps);

private:
    bool enabled_for_map_ = false;
    bool initialized_ = false;
};

} // namespace optsvr

