#ifndef MY_PLANNER
#define MY_PLANNER

// Copied from default_planner and modified
// This is our custom planner based on default_planner implementation

#include "Types.h"
#include "TrajLNS.h"
#include <random>


namespace MyPlanner{

    void initialize(int preprocess_time_limit, SharedEnvironment* env);

    void plan(int time_limit, std::vector<std::vector<Action>> & actions,
                        SharedEnvironment* env, int num_steps=10);
    
}
#endif
