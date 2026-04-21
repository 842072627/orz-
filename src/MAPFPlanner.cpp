#include <random>
#include <Entry.h>

//my planner includes
#include "my_planner/my_planner.h"
#include "my_planner/const.h"

// Default planner (starkit baseline)
#include "planner.h"

// Check if map should use our planner or default starkit planner
// Our planner (MyPlanner) is optimized for fulfill/warehouse but crashes on room/maze/iron/orz
// Default planner (DefaultPlanner) works on all maps but is less optimized
bool should_use_my_planner(SharedEnvironment* env) {
    if (env == nullptr || env->map_name.empty()) return true;
    
    std::string name = env->map_name;
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);
    
    // Use my planner for fulfill/warehouse maps (our optimized planner)
    if (name.find("warehouse") != std::string::npos || 
        name.find("fulfill") != std::string::npos) {
        return true;  // Use our optimized planner
    }
    
    // Use default starkit planner for room, maze, iron, bos, orz, random maps
    return false;
}

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 * 
 * This function call the planner initialize function with a time limit fore preprocessing.
 * 
 * @param preprocess_time_limit The total time limit allocated for preprocessing (in milliseconds).
 */
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    // use the remaining entry time limit (after task scheduling) for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    
    if (should_use_my_planner(env)) {
        MyPlanner::initialize(limit, env);
    } else {
        DefaultPlanner::initialize(limit, env);
    }
    return;
}

/**
 * Plans a path using my planner (based on default planner)
 * 
 * This function performs path planning within the timelimit given, and call the plan function in my planner.
 * The planned actions are output to the provided actions vector.
 * 
 * @param time_limit The time limit allocated for planning (in milliseconds).
 * @param actions A reference to a vector that will be populated with the planned actions (next action for each agent).
 */
void MAPFPlanner::plan(int time_limit, Plan & plan) 
{
    // use the remaining time after task schedule for path planning, -PLANNER_TIMELIMIT_TOLERANCE for timing error tolerance;
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time/(env->action_time*env->max_counter)) + 1;
    
    if (should_use_my_planner(env)) {
        MyPlanner::plan(limit, plan.actions, env, min_plan_steps);
    } else {
        DefaultPlanner::plan(limit, plan.actions, env, min_plan_steps);
    }
    return;
}
