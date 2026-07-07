#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"
#include "orz_highway.h"

// The initialize function will be called by competition system at the preprocessing stage.
// Implement the initialize functions of the planner and scheduler to load or compute auxiliary data.
// Note that, this function runs untill preprocess_time_limit (in milliseconds) is reached.
// This is an offline step, after it completes then evaluation begins.
void Entry::initialize(int preprocess_time_limit)
{
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. a schedule that specifies which agent complete which task.
//  2. a next action that specifies how each agent should move in the next timestep.
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{
    //call the task scheduler to assign tasks to agents
    scheduler->plan(time_limit,proposed_schedule);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);
    
    // idle agent relocation: move idle agents on forbid/guide cells to highway exit
    for (size_t i = 0; i < proposed_schedule.size(); i++) {
        if (proposed_schedule[i] != -1) continue;
        int loc = env->curr_states[i].location;
        uint8_t _fb = ORZ_Highway::get_forbid(loc);
        uint8_t _pd = 0;
        bool _has_guide = ORZ_Highway::get_guide(loc, _pd);
        if (_fb || _has_guide) {
            int exit_loc = -1;
            if (_fb) {
                exit_loc = ORZ_Highway::get_exit(loc);
            } else {
                exit_loc = ORZ_Highway::get_guide_exit(loc);
            }
            if (exit_loc >= 0) {
                env->goal_locations[i].clear();
                env->goal_locations[i].push_back({exit_loc, -1});
            }
        }
    }
    
    //call the planner to compute the actions
    planner->plan(time_limit,plan);

}

// Set the next goal locations for each agent based on the proposed schedule
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // record the proposed schedule so that we can tell the planner
    env->curr_task_schedule = proposed_schedule;

    // The first unfinished errand/location of each task is the next goal for the assigned agent.
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
    return;
}