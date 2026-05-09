#ifndef MY_SCHEDULER
#define MY_SCHEDULER

// Default scheduler baseline.
//
// This is a simple distance-based greedy scheduler: for each free agent, it
// assigns the unassigned task with the smallest estimated completion distance
// (sum of heuristic distances over the task errands in order).

#include "my_Types.h"
#include "SharedEnv.h"
#include "my_heuristics.h"
#include <random>

namespace MyPlanner{

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

}

#endif
