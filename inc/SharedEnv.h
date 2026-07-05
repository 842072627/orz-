#pragma once
#include "States.h"
#include "Grid.h"
#include "nlohmann/json.hpp"
#include "Tasks.h"
#include <unordered_map>
#include <ActionModel.h>


typedef std::chrono::steady_clock::time_point TimePoint;
typedef std::chrono::milliseconds milliseconds;
typedef std::unordered_map<int, Task> TaskPool;

class SharedEnvironment
{
public:
    int num_of_agents;
    int rows;
    int cols;
    std::string map_name;
    std::vector<int> map;
    std::string file_storage_path;
    std::string delay_event_distribution;
    std::string delay_time_distribution;

    // goal locations for each agent
    // each task is a pair of <goal_loc, reveal_time>
    vector< vector<pair<int, int> > > goal_locations;

    int curr_timestep = 0;
    int system_timestep = 0;
    vector<State> curr_states;
    vector<State> start_states;
    vector<State> system_states;

    TaskPool task_pool; // task_id -> Task
    vector<int> new_tasks; // task ids of tasks that are newly revealed in the current timestep
    vector<int> new_freeagents; // agent ids of agents that are newly free in the current timestep
    vector<int> curr_task_schedule; // the current scheduler, agent_id -> task_id

    // plan_start_time records the time point that plan/initialise() function is called; 
    // It is a convenient variable to help planners/schedulers to keep track of time.
    // plan_start_time is updated when the simulation system call the entry plan function, its type is std::chrono::steady_clock::time_point
    TimePoint plan_start_time;

    // the current staged actions in the shared environment
    vector<vector<Action>> staged_actions;

    int min_planner_communication_time; // information of the minimum communication time for the planner to return a plan, in milliseconds. 
    int action_time; // information of the time for executing each action, in milliseconds. 
    int max_counter; // information of the max counter value for agents, which is the number of time ticks that agents have to spend to finish one action

    SharedEnvironment(){}
};

// ─── Scheduler → Planner shared flow data (set by scheduler, read by planner) ───
namespace optsvr {
extern const float* g_sched_ff_dir;       // pointer to scheduler's ff_dir_ data (4*zone_count floats)
extern int   g_sched_ff_zone_rows;       // 40
extern int   g_sched_ff_zone_cols;       // 20
extern int   g_sched_ff_zone_count;      // rows * cols

// Planner → Scheduler shared real-flow data (set by planner after plan, read by scheduler before schedule)
extern float* g_planner_zone_flow;        // [zone_count] real-aggregated flow per zone
extern int   g_planner_zone_flow_count;  // must match g_sched_ff_zone_count

// Planner guide-path traj cache (clean, per-agent zone-level paths)
// Format: flat array [n_agents, agent0_len, z0, z1, ..., agent1_len, z0, z1, ...]
// Set by planner at end of plan(); cleared at start of next plan()
extern int*  g_planner_traj_cache;        // flat int array
extern int   g_planner_traj_cache_size;   // total ints in the array
}
