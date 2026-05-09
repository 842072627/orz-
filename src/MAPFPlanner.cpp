#include <random>
#include <Entry.h>

//my_planner includes
#include "my_planner.h"
#include "my_const.h"

/**
 * Initialises the MAPF planner with a given time limit for preprocessing.
 */
void MAPFPlanner::initialize(int preprocess_time_limit)
{
    int limit = preprocess_time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;
    MyPlanner::initialize(limit, env);
    return;
}

/**
 * Plans a path using my_planner
 */
void MAPFPlanner::plan(int time_limit, Plan & plan) 
{
    int limit = time_limit - std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - env->plan_start_time).count() - MyPlanner::PLANNER_TIMELIMIT_TOLERANCE;

    int min_plan_steps = (int)(env->min_planner_communication_time/(env->action_time*env->max_counter)) + 1;
    MyPlanner::plan(limit, plan.actions, env,min_plan_steps);
    return;
}
